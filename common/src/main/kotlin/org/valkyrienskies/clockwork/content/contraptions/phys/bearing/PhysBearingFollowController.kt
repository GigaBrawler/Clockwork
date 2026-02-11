package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.joml.AxisAngle4d
import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.tanh

internal object PhysBearingFollowController {
    data class JerkLimitedStepState(
        val stepRadPerTick: Double,
        val accelRadPerTick2: Double
    )

    data class FixedAuthorityProfile(
        val maxForce: Double,
        val maxTorque: Double,
        val movingTargetEpsRad: Double,
        val movingDriftForceRad: Double,
        val movingRefreshTicks: Int
    )

    private const val FOLLOW_MOVING_STEP_FULL_SCALE_RAD = 0.6
    private const val FOLLOW_MOVING_ERROR_RAMP_FULL_RAD = 0.2
    private const val FIXED_FORCE_HARD_MAX = 5.0e8
    private const val FIXED_TORQUE_HARD_MAX = 5.0e8
    private const val FOLLOW_MOVING_FORCE_MIN = 2.0e4
    private const val FOLLOW_MOVING_FORCE_MAX = 1.5e7
    private const val FOLLOW_MOVING_TORQUE_MIN = 2.0e4
    private const val FOLLOW_MOVING_TORQUE_MAX = 1.2e7
    private const val LOCKED_HOLD_FORCE_MAX = 2.0e8
    private const val LOCKED_HOLD_TORQUE_MAX = 2.0e8
    private const val DYNAMIC_MAIN_FORCE_SUPPRESSION_FLOOR = 0.01
    private const val DYNAMIC_MAIN_TORQUE_SUPPRESSION_FLOOR = 0.08
    private const val MOVING_TARGET_EPS_BASE_RAD = 0.004
    private const val MOVING_DRIFT_FORCE_BASE_RAD = 0.012
    private const val MOVING_REFRESH_TICKS_BASE = 2
    private const val PROFILE_MIN_CAP = 2.0e4

    fun isFixedJointMode(modeName: String, aligning: Boolean): Boolean {
        return aligning || modeName == "FOLLOW_ANGLE" || modeName == "LOCKED"
    }

    fun shouldForceMovingFollowUpdate(modeName: String, aligning: Boolean, commandedSpeed: Float): Boolean {
        return modeName == "FOLLOW_ANGLE" && !aligning && commandedSpeed.isFinite() && abs(commandedSpeed) > 0.0f
    }

    fun isFollowCommandActive(modeName: String, aligning: Boolean, commandedSpeed: Float, speedEps: Float): Boolean {
        if (modeName != "FOLLOW_ANGLE" || aligning) return false
        if (!commandedSpeed.isFinite()) return false
        return abs(commandedSpeed) >= abs(speedEps)
    }

    fun shouldLatchFollowStop(
        modeName: String,
        aligning: Boolean,
        wasMovingFollow: Boolean,
        followCommandActive: Boolean
    ): Boolean {
        return modeName == "FOLLOW_ANGLE" && !aligning && wasMovingFollow && !followCommandActive
    }

    fun normalizeDisplayAngleDeg720(angleDeg: Double): Float {
        if (!angleDeg.isFinite()) return 0f
        var normalized = angleDeg % 720.0
        if (normalized < 0.0) normalized += 720.0
        return normalized.toFloat()
    }

    fun computeAdaptiveCatchupGain(
        errorAbsRad: Double,
        minGain: Double,
        maxGain: Double,
        fullErrorRad: Double
    ): Double {
        if (!errorAbsRad.isFinite()) return 0.0
        val min = minGain.coerceAtLeast(0.0)
        val max = maxGain.coerceAtLeast(min)
        val full = fullErrorRad.takeIf { it.isFinite() && it > 0.0 } ?: return max
        val blend = (errorAbsRad / full).coerceIn(0.0, 1.0)
        return min + (max - min) * blend
    }

    fun normalizeAngleErrorRad(targetAngleRad: Double, currentAngleRad: Double): Double {
        if (!targetAngleRad.isFinite() || !currentAngleRad.isFinite()) return 0.0
        val d = targetAngleRad - currentAngleRad
        return atan2(sin(d), cos(d))
    }

    fun wrapAngleSignedPiRad(angleRad: Double): Double {
        if (!angleRad.isFinite()) return 0.0
        return atan2(sin(angleRad), cos(angleRad))
    }

    fun unwrapAngleNearReferenceRad(referenceUnwrappedRad: Double, wrappedTargetRad: Double): Double {
        if (!wrappedTargetRad.isFinite()) return referenceUnwrappedRad
        if (!referenceUnwrappedRad.isFinite()) return wrapAngleSignedPiRad(wrappedTargetRad)
        val error = normalizeAngleErrorRad(wrappedTargetRad, referenceUnwrappedRad)
        return referenceUnwrappedRad + error
    }

    fun selectDesiredContinuousTarget(
        wrappedTargetRad: Double,
        measuredAngleRad: Double,
        previousDesiredContinuousRad: Double?,
        inFollowSettleWindow: Boolean
    ): Double {
        if (inFollowSettleWindow && measuredAngleRad.isFinite()) return measuredAngleRad
        val unwrapRef = previousDesiredContinuousRad ?: measuredAngleRad
        return unwrapAngleNearReferenceRad(unwrapRef, wrappedTargetRad)
    }

    fun normalizedSnapDeltaRad(targetAngleRad: Double, currentAngleRad: Double): Double {
        return normalizeAngleErrorRad(targetAngleRad, currentAngleRad)
    }

    fun bootstrapFixedTargetAngleRad(currentAngleRad: Double?, desiredTargetRad: Double): Double {
        if (!desiredTargetRad.isFinite()) return 0.0
        return if (currentAngleRad != null && currentAngleRad.isFinite()) currentAngleRad else desiredTargetRad
    }

    fun stepTowardAngleRad(currentAngleRad: Double, targetAngleRad: Double, maxStepRad: Double): Double {
        if (!currentAngleRad.isFinite() || !targetAngleRad.isFinite()) return currentAngleRad
        val stepLimit = abs(maxStepRad)
        if (stepLimit <= 0.0) return currentAngleRad
        val error = normalizeAngleErrorRad(targetAngleRad, currentAngleRad)
        if (!error.isFinite()) return currentAngleRad
        if (abs(error) <= stepLimit) return currentAngleRad + error
        return currentAngleRad + error.coerceIn(-stepLimit, stepLimit)
    }

    fun computeFollowTrackMaxStepRad(
        commandedStepRad: Double,
        minStepRad: Double,
        maxStepRad: Double,
        gain: Double
    ): Double {
        val min = abs(minStepRad)
        val max = abs(maxStepRad).coerceAtLeast(min)
        val raw = if (commandedStepRad.isFinite() && gain.isFinite()) abs(commandedStepRad) * abs(gain) else min
        return raw.coerceIn(min, max)
    }

    fun stepJerkLimitedCommand(
        currentStepRadPerTick: Double,
        currentAccelRadPerTick2: Double,
        commandedStepRadPerTick: Double,
        maxAccelRadPerTick2: Double,
        maxJerkRadPerTick3: Double,
        maxAbsStepRadPerTick: Double
    ): JerkLimitedStepState {
        if (!currentStepRadPerTick.isFinite() || !currentAccelRadPerTick2.isFinite() || !commandedStepRadPerTick.isFinite()) {
            return JerkLimitedStepState(0.0, 0.0)
        }
        val accelCap = abs(maxAccelRadPerTick2)
        val jerkCap = abs(maxJerkRadPerTick3)
        val stepCap = abs(maxAbsStepRadPerTick).coerceAtLeast(0.0)

        val stepDelta = commandedStepRadPerTick - currentStepRadPerTick
        val desiredAccel = stepDelta.coerceIn(-accelCap, accelCap)
        val accelDelta = desiredAccel - currentAccelRadPerTick2
        val nextAccel = (currentAccelRadPerTick2 + accelDelta.coerceIn(-jerkCap, jerkCap)).coerceIn(-accelCap, accelCap)
        val nextStep = (currentStepRadPerTick + nextAccel).coerceIn(-stepCap, stepCap)
        return JerkLimitedStepState(nextStep, nextAccel)
    }

    fun computePostLoadAuthorityRamp(
        totalSettleTicks: Int,
        settleTicksRemaining: Int,
        minMultiplier: Double
    ): Double {
        if (totalSettleTicks <= 0) return 1.0
        val min = minMultiplier.coerceIn(0.0, 1.0)
        val remaining = settleTicksRemaining.coerceIn(0, totalSettleTicks)
        val progress = 1.0 - (remaining.toDouble() / totalSettleTicks.toDouble())
        return (min + (1.0 - min) * progress).coerceIn(min, 1.0)
    }

    fun computeFixedAuthorityProfile(
        modeName: String,
        aligning: Boolean,
        movingFollow: Boolean,
        inPostLoadSettle: Boolean,
        subMass: Double?,
        mainMass: Double?,
        commandedStepMagnitudeRad: Double,
        trackingErrorAbsRad: Double = 0.0,
        postLoadAuthorityMultiplier: Double
    ): FixedAuthorityProfile {
        val lockedLikeHold = aligning || modeName == "LOCKED" || !movingFollow
        val stepBlend = if (commandedStepMagnitudeRad.isFinite()) {
            (abs(commandedStepMagnitudeRad) / FOLLOW_MOVING_STEP_FULL_SCALE_RAD).coerceIn(0.0, 1.0)
        } else {
            0.0
        }
        val errorBlend = if (trackingErrorAbsRad.isFinite()) {
            (abs(trackingErrorAbsRad) / FOLLOW_MOVING_ERROR_RAMP_FULL_RAD).coerceIn(0.0, 1.0)
        } else {
            0.0
        }
        val errorRamp = 0.2 + 0.8 * tanh(errorBlend)

        var forceCap = if (lockedLikeHold) {
            LOCKED_HOLD_FORCE_MAX
        } else {
            FOLLOW_MOVING_FORCE_MIN + (FOLLOW_MOVING_FORCE_MAX - FOLLOW_MOVING_FORCE_MIN) * stepBlend
        }
        var torqueCap = if (lockedLikeHold) {
            LOCKED_HOLD_TORQUE_MAX
        } else {
            FOLLOW_MOVING_TORQUE_MIN + (FOLLOW_MOVING_TORQUE_MAX - FOLLOW_MOVING_TORQUE_MIN) * stepBlend
        }
        if (!lockedLikeHold) {
            forceCap = FOLLOW_MOVING_FORCE_MIN + (forceCap - FOLLOW_MOVING_FORCE_MIN) * errorRamp
            torqueCap = FOLLOW_MOVING_TORQUE_MIN + (torqueCap - FOLLOW_MOVING_TORQUE_MIN) * errorRamp
        }

        var forceSuppressionScale = 1.0
        var torqueSuppressionScale = 1.0
        if (!lockedLikeHold && movingFollow) {
            val sub = subMass?.takeIf { it.isFinite() && it > 0.0 }
            val main = mainMass?.takeIf { it.isFinite() && it > 0.0 }
            if (sub != null && main != null) {
                val massRatio = (sub / (sub + main)).coerceIn(0.0, 1.0)
                forceSuppressionScale = massRatio.pow(0.75).coerceIn(DYNAMIC_MAIN_FORCE_SUPPRESSION_FLOOR, 1.0)
                torqueSuppressionScale = (
                    DYNAMIC_MAIN_TORQUE_SUPPRESSION_FLOOR +
                        (1.0 - DYNAMIC_MAIN_TORQUE_SUPPRESSION_FLOOR) * massRatio.pow(0.55)
                    ).coerceIn(DYNAMIC_MAIN_TORQUE_SUPPRESSION_FLOOR, 1.0)
                forceCap *= forceSuppressionScale
                torqueCap *= torqueSuppressionScale
            }
        }

        if (inPostLoadSettle) {
            val ramp = postLoadAuthorityMultiplier.coerceIn(0.05, 1.0)
            forceCap *= ramp
            torqueCap *= ramp
        }

        forceCap = forceCap.coerceIn(PROFILE_MIN_CAP, FIXED_FORCE_HARD_MAX)
        torqueCap = torqueCap.coerceIn(PROFILE_MIN_CAP, FIXED_TORQUE_HARD_MAX)

        val epsScale = if (!lockedLikeHold && movingFollow) {
            1.0 / forceSuppressionScale.coerceAtLeast(DYNAMIC_MAIN_FORCE_SUPPRESSION_FLOOR)
        } else {
            1.0
        }
        val movingTargetEpsRad = (MOVING_TARGET_EPS_BASE_RAD * epsScale).coerceIn(MOVING_TARGET_EPS_BASE_RAD, 0.03)
        val movingDriftForceRad = (MOVING_DRIFT_FORCE_BASE_RAD * epsScale).coerceIn(MOVING_DRIFT_FORCE_BASE_RAD, 0.05)
        val movingRefreshTicks = if (!lockedLikeHold && movingFollow) {
            (MOVING_REFRESH_TICKS_BASE * epsScale).roundToInt().coerceIn(MOVING_REFRESH_TICKS_BASE, 8)
        } else {
            MOVING_REFRESH_TICKS_BASE
        }

        return FixedAuthorityProfile(
            maxForce = forceCap,
            maxTorque = torqueCap,
            movingTargetEpsRad = movingTargetEpsRad,
            movingDriftForceRad = movingDriftForceRad,
            movingRefreshTicks = movingRefreshTicks
        )
    }

    fun enforceQuaternionHemisphere(candidate: Quaterniondc, reference: Quaterniondc): Quaterniond {
        val stabilized = Quaterniond(candidate).normalize()
        if (!stabilized.x().isFinite() || !stabilized.y().isFinite() || !stabilized.z().isFinite() || !stabilized.w().isFinite()) {
            return stabilized
        }

        if (!reference.x().isFinite() || !reference.y().isFinite() || !reference.z().isFinite() || !reference.w().isFinite()) {
            return stabilized
        }

        return if (stabilized.dot(reference) < 0.0) {
            stabilized.mul(-1.0)
        } else {
            stabilized
        }
    }

    fun captureZeroRelRotation(
        relMainToSubNow: Quaterniondc,
        axisMainLocal: Vector3dc,
        currentAngleRad: Double
    ): Quaterniond {
        val relNow = Quaterniond(relMainToSubNow).normalize()
        if (!currentAngleRad.isFinite()) return relNow
        val axis = axisMainLocal.get(Vector3d())
        if (!axis.x.isFinite() || !axis.y.isFinite() || !axis.z.isFinite()) return relNow
        if (axis.lengthSquared() < 1.0e-12) return relNow
        axis.normalize()
        return Quaterniond(AxisAngle4d(-currentAngleRad, axis)).mul(relNow, Quaterniond()).normalize()
    }

    fun desiredRelRotationFromZeroRef(
        zeroRefMainToSub: Quaterniondc,
        axisMainLocal: Vector3dc,
        targetAngleRad: Double
    ): Quaterniond {
        val baseRef = Quaterniond(zeroRefMainToSub).normalize()
        if (!targetAngleRad.isFinite()) return baseRef
        val axis = axisMainLocal.get(Vector3d())
        if (!axis.x.isFinite() || !axis.y.isFinite() || !axis.z.isFinite()) return baseRef
        if (axis.lengthSquared() < 1.0e-12) return baseRef
        axis.normalize()
        return Quaterniond(AxisAngle4d(targetAngleRad, axis)).mul(baseRef, Quaterniond()).normalize()
    }

    fun hasValidFixedReferenceContext(
        currentAngleRad: Double?,
        referenceCaptured: Boolean,
        zeroRefMainToSub: Quaterniondc?,
        anchorCaptured: Boolean,
        anchorPose0Local: Vector3dc?,
        anchorPose1Local: Vector3dc?
    ): Boolean {
        val ref = zeroRefMainToSub ?: return false
        if (!referenceCaptured) return false
        if (currentAngleRad == null || !currentAngleRad.isFinite()) return false
        val pose0 = anchorPose0Local ?: return false
        val pose1 = anchorPose1Local ?: return false
        if (!anchorCaptured) return false
        return ref.x().isFinite() &&
            ref.y().isFinite() &&
            ref.z().isFinite() &&
            ref.w().isFinite() &&
            pose0.x().isFinite() &&
            pose0.y().isFinite() &&
            pose0.z().isFinite() &&
            pose1.x().isFinite() &&
            pose1.y().isFinite() &&
            pose1.z().isFinite()
    }

    fun shouldApplyFixedTargetUpdate(
        modeOrAlignmentTransition: Boolean,
        jointKindMismatch: Boolean,
        targetDeltaAbsRad: Double,
        driftAbsRad: Double,
        targetEpsRad: Double,
        holdDriftDeadbandRad: Double,
        ticksSinceLastRefresh: Int,
        safetyRefreshTicks: Int
    ): Boolean {
        if (modeOrAlignmentTransition || jointKindMismatch) return true

        if (!targetDeltaAbsRad.isFinite() || !driftAbsRad.isFinite()) return true
        if (targetDeltaAbsRad >= targetEpsRad) return true
        if (driftAbsRad > holdDriftDeadbandRad) return true

        // Safety refresh must not pulse at rest: require measurable drift to re-apply.
        if (ticksSinceLastRefresh >= safetyRefreshTicks && driftAbsRad > holdDriftDeadbandRad) return true
        return false
    }

    fun shouldApplyMovingFixedTargetUpdate(
        movingFollow: Boolean,
        inFollowSettleWindow: Boolean,
        referenceContextValid: Boolean,
        commandedStepRadPerTick: Double,
        movingCommandActiveStepRad: Double,
        modeOrAlignmentTransition: Boolean,
        jointKindMismatch: Boolean,
        targetDeltaAbsRad: Double,
        driftAbsRad: Double,
        targetEpsRad: Double,
        movingTargetEpsRad: Double,
        holdDriftDeadbandRad: Double,
        movingDriftForceRad: Double,
        ticksSinceLastRefresh: Int,
        safetyRefreshTicks: Int
    ): Boolean {
        if (!movingFollow || inFollowSettleWindow) {
            return shouldApplyFixedTargetUpdate(
                modeOrAlignmentTransition = modeOrAlignmentTransition,
                jointKindMismatch = jointKindMismatch,
                targetDeltaAbsRad = targetDeltaAbsRad,
                driftAbsRad = driftAbsRad,
                targetEpsRad = targetEpsRad,
                holdDriftDeadbandRad = holdDriftDeadbandRad,
                ticksSinceLastRefresh = ticksSinceLastRefresh,
                safetyRefreshTicks = safetyRefreshTicks
            )
        }

        if (modeOrAlignmentTransition || jointKindMismatch) return true
        if (!referenceContextValid) return false
        if (!commandedStepRadPerTick.isFinite()) return true
        return abs(commandedStepRadPerTick) >= abs(movingCommandActiveStepRad)
    }
}
