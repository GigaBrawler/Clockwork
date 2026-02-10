package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.joml.AxisAngle4d
import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

internal object PhysBearingFollowController {
    fun isFixedJointMode(modeName: String, aligning: Boolean): Boolean {
        return aligning || modeName == "FOLLOW_ANGLE" || modeName == "LOCKED"
    }

    fun shouldForceMovingFollowUpdate(modeName: String, aligning: Boolean, commandedSpeed: Float): Boolean {
        return modeName == "FOLLOW_ANGLE" && !aligning && commandedSpeed.isFinite() && abs(commandedSpeed) > 0.0f
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
        modeOrAlignmentTransition: Boolean,
        jointKindMismatch: Boolean,
        targetDeltaAbsRad: Double,
        driftAbsRad: Double,
        targetEpsRad: Double,
        movingTargetEpsRad: Double,
        holdDriftDeadbandRad: Double,
        movingDriftForceRad: Double,
        ticksSinceLastRefresh: Int,
        safetyRefreshTicks: Int,
        movingRefreshTicks: Int
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
        if (!targetDeltaAbsRad.isFinite() || !driftAbsRad.isFinite()) return true
        if (targetDeltaAbsRad >= movingTargetEpsRad) return true
        if (driftAbsRad > movingDriftForceRad) return true
        if (ticksSinceLastRefresh >= movingRefreshTicks) return true
        return false
    }
}
