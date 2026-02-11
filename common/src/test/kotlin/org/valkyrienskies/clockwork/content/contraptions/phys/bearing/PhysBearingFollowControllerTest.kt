package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.joml.AxisAngle4d
import org.joml.Quaterniond
import org.joml.Vector3d
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class PhysBearingFollowControllerTest {
    private fun quatClose(a: Quaterniond, b: Quaterniond, eps: Double = 1.0e-9): Boolean {
        val dot = abs(a.dot(b))
        return abs(1.0 - dot) <= eps
    }

    @Test
    fun fixedModeSelectionMatchesContract() {
        assertFalse(PhysBearingFollowController.isFixedJointMode("UNLOCKED", aligning = false))
        assertTrue(PhysBearingFollowController.isFixedJointMode("UNLOCKED", aligning = true))

        assertTrue(PhysBearingFollowController.isFixedJointMode("FOLLOW_ANGLE", aligning = false))
        assertTrue(PhysBearingFollowController.isFixedJointMode("FOLLOW_ANGLE", aligning = true))

        assertTrue(PhysBearingFollowController.isFixedJointMode("LOCKED", aligning = false))
        assertTrue(PhysBearingFollowController.isFixedJointMode("LOCKED", aligning = true))
    }

    @Test
    fun snapDeltaUsesShortestPath() {
        val nearWrapPositive = PhysBearingFollowController.normalizeAngleErrorRad(
            targetAngleRad = Math.toRadians(350.0),
            currentAngleRad = Math.toRadians(10.0)
        )
        val nearWrapNegative = PhysBearingFollowController.normalizeAngleErrorRad(
            targetAngleRad = Math.toRadians(10.0),
            currentAngleRad = Math.toRadians(350.0)
        )
        assertTrue(nearWrapPositive < 0.0)
        assertTrue(nearWrapNegative > 0.0)
        assertTrue(abs(nearWrapPositive) < Math.PI)
        assertTrue(abs(nearWrapNegative) < Math.PI)
    }

    @Test
    fun stepTowardAngleUsesShortestPathAndStepCap() {
        val step = PhysBearingFollowController.stepTowardAngleRad(
            currentAngleRad = Math.toRadians(170.0),
            targetAngleRad = Math.toRadians(-170.0),
            maxStepRad = Math.toRadians(4.0)
        )
        val delta = PhysBearingFollowController.normalizeAngleErrorRad(step, Math.toRadians(170.0))
        assertTrue(delta > 0.0)
        assertTrue(abs(delta) <= Math.toRadians(4.0) + 1.0e-12)
    }

    @Test
    fun unwrapAngleNearReferenceIsContinuousAcross2pi() {
        val reference = 2.0 * Math.PI - 0.02
        val wrappedTarget = 0.01
        val unwrapped = PhysBearingFollowController.unwrapAngleNearReferenceRad(reference, wrappedTarget)
        assertTrue(unwrapped > reference)
        assertTrue(abs(unwrapped - reference) < 0.05)
    }

    @Test
    fun desired_unwrap_uses_previous_desired_not_active() {
        val previousDesired = 4.0 * Math.PI + 0.02
        val laggedActive = 2.0 * Math.PI + 0.2
        val wrappedTarget = 0.03

        val desiredFromPrevious = PhysBearingFollowController.selectDesiredContinuousTarget(
            wrappedTargetRad = wrappedTarget,
            measuredAngleRad = laggedActive,
            previousDesiredContinuousRad = previousDesired,
            inFollowSettleWindow = false
        )
        val desiredFromActive = PhysBearingFollowController.unwrapAngleNearReferenceRad(laggedActive, wrappedTarget)

        assertTrue(abs(desiredFromPrevious - previousDesired) < 0.05)
        assertTrue(abs(desiredFromPrevious - desiredFromActive) > Math.PI)
    }

    @Test
    fun desired_unwrap_no_branch_flip_under_active_lag() {
        val previousDesired = 6.0 * Math.PI - 0.01
        val laggedActive = 4.0 * Math.PI + 1.2
        val wrappedTarget = 0.01

        val desired = PhysBearingFollowController.selectDesiredContinuousTarget(
            wrappedTargetRad = wrappedTarget,
            measuredAngleRad = laggedActive,
            previousDesiredContinuousRad = previousDesired,
            inFollowSettleWindow = false
        )

        assertTrue(desired > previousDesired)
        assertTrue(abs(desired - previousDesired) < 0.05)
    }

    @Test
    fun stepTowardDoesNotSnapToWrappedEquivalent() {
        val current = 2.0 * Math.PI - 0.01
        val wrappedTarget = 0.0
        val step = PhysBearingFollowController.stepTowardAngleRad(
            currentAngleRad = current,
            targetAngleRad = wrappedTarget,
            maxStepRad = 0.1
        )
        assertTrue(step > Math.PI)
        assertTrue(abs(PhysBearingFollowController.normalizeAngleErrorRad(wrappedTarget, step)) < 0.02)
    }

    @Test
    fun follow_step_bounded_but_continuous() {
        val commandedStep = Math.toRadians(12.0)
        val maxStep = PhysBearingFollowController.computeFollowTrackMaxStepRad(
            commandedStepRad = commandedStep,
            minStepRad = 0.01,
            maxStepRad = 1.2,
            gain = 1.35
        )
        assertTrue(maxStep in 0.01..1.2)

        val current = 3.0 * Math.PI
        val target = current + 0.15
        val stepped = PhysBearingFollowController.stepTowardAngleRad(current, target, maxStep)
        val applied = abs(PhysBearingFollowController.normalizeAngleErrorRad(stepped, current))
        assertTrue(applied <= maxStep + 1.0e-12)
    }

    @Test
    fun lockedTransitionSmoothToTargetNoSnap() {
        val current = Math.toRadians(0.0)
        val target = Math.toRadians(120.0)
        val stepped = PhysBearingFollowController.stepTowardAngleRad(
            currentAngleRad = current,
            targetAngleRad = target,
            maxStepRad = Math.toRadians(6.0)
        )
        assertTrue(abs(PhysBearingFollowController.normalizeAngleErrorRad(stepped, current)) <= Math.toRadians(6.0) + 1.0e-12)
        assertTrue(abs(PhysBearingFollowController.normalizeAngleErrorRad(target, stepped)) > 0.0)
    }

    @Test
    fun fixedEntryBootstrapCapturesCurrentAngle() {
        val currentAngle = Math.toRadians(47.5)
        val desired = Math.toRadians(0.0)
        val bootstrapped = PhysBearingFollowController.bootstrapFixedTargetAngleRad(currentAngle, desired)
        assertTrue(abs(bootstrapped - currentAngle) < 1.0e-12)
    }

    @Test
    fun captureZeroReferenceRemovesCurrentAngleTwist() {
        val axis = Vector3d(0.0, 1.0, 0.0)
        val current = Math.toRadians(55.0)
        val drift = Quaterniond(AxisAngle4d(Math.toRadians(12.0), Vector3d(1.0, 0.0, 0.0)))
        val relNow = Quaterniond(AxisAngle4d(current, axis)).mul(drift, Quaterniond()).normalize()

        val zeroRef = PhysBearingFollowController.captureZeroRelRotation(relNow, axis, current)
        assertTrue(quatClose(zeroRef, drift))
    }

    @Test
    fun desiredRelFromZeroRefDependsOnlyOnTargetNotCurrentDrift() {
        val axis = Vector3d(0.0, 0.0, 1.0)
        val current = Math.toRadians(30.0)
        val target = Math.toRadians(100.0)
        val drift = Quaterniond(AxisAngle4d(Math.toRadians(18.0), Vector3d(1.0, 0.0, 0.0)))
        val relNow = Quaterniond(AxisAngle4d(current, axis)).mul(drift, Quaterniond()).normalize()

        val zeroRef = PhysBearingFollowController.captureZeroRelRotation(relNow, axis, current)
        val desired = PhysBearingFollowController.desiredRelRotationFromZeroRef(zeroRef, axis, target)
        val expected = Quaterniond(AxisAngle4d(target, axis)).mul(drift, Quaterniond()).normalize()
        assertTrue(quatClose(desired, expected))
    }

    @Test
    fun hemisphereContinuityPreservesQuaternionSignConsistency() {
        val reference = Quaterniond(0.0, 0.0, 0.0, 1.0)
        val candidate = Quaterniond(0.0, 0.0, 0.0, -1.0)
        val stabilized = PhysBearingFollowController.enforceQuaternionHemisphere(candidate, reference)
        assertTrue(stabilized.dot(reference) > 0.0)
    }

    @Test
    fun fixedUpdateRequiresValidReferenceContext() {
        val ref = Quaterniond()
        val anchor0 = Vector3d(1.0, 2.0, 3.0)
        val anchor1 = Vector3d(4.0, 5.0, 6.0)
        assertFalse(PhysBearingFollowController.hasValidFixedReferenceContext(null, true, ref, true, anchor0, anchor1))
        assertFalse(PhysBearingFollowController.hasValidFixedReferenceContext(0.0, false, ref, true, anchor0, anchor1))
        assertFalse(PhysBearingFollowController.hasValidFixedReferenceContext(0.0, true, null, true, anchor0, anchor1))
        assertFalse(PhysBearingFollowController.hasValidFixedReferenceContext(0.0, true, ref, false, anchor0, anchor1))
        assertFalse(PhysBearingFollowController.hasValidFixedReferenceContext(0.0, true, ref, true, null, anchor1))
        assertFalse(PhysBearingFollowController.hasValidFixedReferenceContext(0.0, true, ref, true, anchor0, null))
        assertTrue(PhysBearingFollowController.hasValidFixedReferenceContext(0.0, true, ref, true, anchor0, anchor1))
    }

    @Test
    fun refreshRequiresDeltaOrDriftNoPeriodicPulse() {
        assertFalse(
            PhysBearingFollowController.shouldApplyFixedTargetUpdate(
                modeOrAlignmentTransition = false,
                jointKindMismatch = false,
                targetDeltaAbsRad = 5.0e-5,
                driftAbsRad = 5.0e-5,
                targetEpsRad = 1.0e-3,
                holdDriftDeadbandRad = 1.0e-3,
                ticksSinceLastRefresh = 10_000,
                safetyRefreshTicks = 200
            )
        )

        assertTrue(
            PhysBearingFollowController.shouldApplyFixedTargetUpdate(
                modeOrAlignmentTransition = false,
                jointKindMismatch = false,
                targetDeltaAbsRad = 2.0e-3,
                driftAbsRad = 0.0,
                targetEpsRad = 1.0e-3,
                holdDriftDeadbandRad = 1.0e-3,
                ticksSinceLastRefresh = 1,
                safetyRefreshTicks = 200
            )
        )

        assertTrue(
            PhysBearingFollowController.shouldApplyFixedTargetUpdate(
                modeOrAlignmentTransition = false,
                jointKindMismatch = false,
                targetDeltaAbsRad = 0.0,
                driftAbsRad = 2.0e-3,
                targetEpsRad = 1.0e-3,
                holdDriftDeadbandRad = 1.0e-3,
                ticksSinceLastRefresh = 1,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun moving_scheduler_streams_when_command_active() {
        assertTrue(
            PhysBearingFollowController.shouldApplyMovingFixedTargetUpdate(
                movingFollow = true,
                inFollowSettleWindow = false,
                referenceContextValid = true,
                commandedStepRadPerTick = 1.0e-3,
                movingCommandActiveStepRad = 5.0e-5,
                modeOrAlignmentTransition = false,
                jointKindMismatch = false,
                targetDeltaAbsRad = 1.0e-4,
                driftAbsRad = 1.0e-4,
                targetEpsRad = 1.0e-3,
                movingTargetEpsRad = 0.004,
                holdDriftDeadbandRad = 1.0e-3,
                movingDriftForceRad = 0.012,
                ticksSinceLastRefresh = 1,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun moving_scheduler_no_pulse_when_not_moving() {
        assertFalse(
            PhysBearingFollowController.shouldApplyMovingFixedTargetUpdate(
                movingFollow = true,
                inFollowSettleWindow = false,
                referenceContextValid = true,
                commandedStepRadPerTick = 1.0e-6,
                movingCommandActiveStepRad = 5.0e-5,
                modeOrAlignmentTransition = false,
                jointKindMismatch = false,
                targetDeltaAbsRad = 1.0e-4,
                driftAbsRad = 1.0e-4,
                targetEpsRad = 1.0e-3,
                movingTargetEpsRad = 0.004,
                holdDriftDeadbandRad = 1.0e-3,
                movingDriftForceRad = 0.012,
                ticksSinceLastRefresh = 1,
                safetyRefreshTicks = 200
            )
        )

        assertFalse(
            PhysBearingFollowController.shouldApplyMovingFixedTargetUpdate(
                movingFollow = true,
                inFollowSettleWindow = false,
                referenceContextValid = true,
                commandedStepRadPerTick = 1.0e-6,
                movingCommandActiveStepRad = 5.0e-5,
                modeOrAlignmentTransition = false,
                jointKindMismatch = false,
                targetDeltaAbsRad = 1.0e-4,
                driftAbsRad = 1.0e-4,
                targetEpsRad = 1.0e-3,
                movingTargetEpsRad = 0.004,
                holdDriftDeadbandRad = 1.0e-3,
                movingDriftForceRad = 0.012,
                ticksSinceLastRefresh = 2,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun moving_scheduler_still_forces_on_transition() {
        assertTrue(
            PhysBearingFollowController.shouldApplyMovingFixedTargetUpdate(
                movingFollow = true,
                inFollowSettleWindow = false,
                referenceContextValid = true,
                commandedStepRadPerTick = 1.0e-6,
                movingCommandActiveStepRad = 5.0e-5,
                modeOrAlignmentTransition = true,
                jointKindMismatch = false,
                targetDeltaAbsRad = 1.0e-6,
                driftAbsRad = 1.0e-4,
                targetEpsRad = 1.0e-3,
                movingTargetEpsRad = 0.004,
                holdDriftDeadbandRad = 1.0e-3,
                movingDriftForceRad = 0.012,
                ticksSinceLastRefresh = 1,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun moving_scheduler_still_forces_on_mismatch() {
        assertTrue(
            PhysBearingFollowController.shouldApplyMovingFixedTargetUpdate(
                movingFollow = true,
                inFollowSettleWindow = false,
                referenceContextValid = true,
                commandedStepRadPerTick = 1.0e-6,
                movingCommandActiveStepRad = 5.0e-5,
                modeOrAlignmentTransition = false,
                jointKindMismatch = true,
                targetDeltaAbsRad = 1.0e-4,
                driftAbsRad = 1.0e-4,
                targetEpsRad = 1.0e-3,
                movingTargetEpsRad = 0.004,
                holdDriftDeadbandRad = 1.0e-3,
                movingDriftForceRad = 0.012,
                ticksSinceLastRefresh = 1,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun post_load_settle_holds_to_measured_before_tracking() {
        val measured = 2.4
        val desiredWrapped = 5.9
        val selected = PhysBearingFollowController.selectDesiredContinuousTarget(
            wrappedTargetRad = desiredWrapped,
            measuredAngleRad = measured,
            previousDesiredContinuousRad = 11.8,
            inFollowSettleWindow = true
        )
        assertTrue(abs(selected - measured) < 1.0e-12)
    }

    @Test
    fun jerk_limited_step_monotonic_no_burst() {
        var step = 0.0
        var accel = 0.0
        repeat(8) {
            val state = PhysBearingFollowController.stepJerkLimitedCommand(
                currentStepRadPerTick = step,
                currentAccelRadPerTick2 = accel,
                commandedStepRadPerTick = 0.4,
                maxAccelRadPerTick2 = 0.08,
                maxJerkRadPerTick3 = 0.20,
                maxAbsStepRadPerTick = 1.2
            )
            assertTrue(state.stepRadPerTick >= step)
            assertTrue(abs(state.stepRadPerTick - step) <= 0.08 + 1.0e-12)
            step = state.stepRadPerTick
            accel = state.accelRadPerTick2
        }
        assertTrue(step <= 0.4 + 1.0e-9)
    }

    @Test
    fun dynamic_authority_profile_reduces_caps_for_heavy_main_dynamic() {
        val profile = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        assertTrue(profile.maxForce in 2.0e4..1.5e7)
        assertTrue(profile.maxTorque in 2.0e4..1.2e7)
        assertTrue(profile.movingTargetEpsRad >= 0.004)
        assertTrue(profile.movingDriftForceRad >= 0.012)
    }

    @Test
    fun dynamic_authority_profile_world_mount_not_over_suppressed() {
        val worldMounted = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = null,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        val dynamicMain = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        assertTrue(worldMounted.maxForce >= dynamicMain.maxForce)
        assertTrue(worldMounted.maxTorque >= dynamicMain.maxTorque)
    }

    @Test
    fun authority_profile_force_torque_scales_with_mass_ratio() {
        val lowRatio = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        val highRatio = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 2_000_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        assertTrue(lowRatio.maxForce <= highRatio.maxForce)
        assertTrue(lowRatio.maxTorque <= highRatio.maxTorque)
    }

    @Test
    fun authority_profile_error_ramp_monotonic() {
        val lowError = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 1_000_000.0,
            mainMass = 1_000_000.0,
            commandedStepMagnitudeRad = 0.2,
            trackingErrorAbsRad = 0.001,
            postLoadAuthorityMultiplier = 1.0
        )
        val highError = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 1_000_000.0,
            mainMass = 1_000_000.0,
            commandedStepMagnitudeRad = 0.2,
            trackingErrorAbsRad = 0.3,
            postLoadAuthorityMultiplier = 1.0
        )
        assertTrue(lowError.maxForce <= highError.maxForce)
        assertTrue(lowError.maxTorque <= highError.maxTorque)
    }

    @Test
    fun post_load_authority_ramp_is_monotonic_and_bounded() {
        val total = 12
        val min = 0.25
        val early = PhysBearingFollowController.computePostLoadAuthorityRamp(total, 12, min)
        val mid = PhysBearingFollowController.computePostLoadAuthorityRamp(total, 6, min)
        val late = PhysBearingFollowController.computePostLoadAuthorityRamp(total, 0, min)
        assertTrue(early in min..1.0)
        assertTrue(mid in min..1.0)
        assertTrue(late in min..1.0)
        assertTrue(early <= mid)
        assertTrue(mid <= late)
    }

    @Test
    fun authority_profile_outputs_finite_values() {
        val profile = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "LOCKED",
            aligning = false,
            movingFollow = false,
            inPostLoadSettle = true,
            subMass = 1.0,
            mainMass = 1.0,
            commandedStepMagnitudeRad = 0.0,
            postLoadAuthorityMultiplier = 0.4
        )
        assertTrue(profile.maxForce.isFinite())
        assertTrue(profile.maxTorque.isFinite())
        assertTrue(profile.movingTargetEpsRad.isFinite())
        assertTrue(profile.movingDriftForceRad.isFinite())
        assertTrue(profile.movingRefreshTicks >= 1)
    }

    @Test
    fun movingFollowRequestsTickUpdates() {
        assertTrue(PhysBearingFollowController.shouldForceMovingFollowUpdate("FOLLOW_ANGLE", aligning = false, commandedSpeed = 1.0f))
        assertFalse(PhysBearingFollowController.shouldForceMovingFollowUpdate("FOLLOW_ANGLE", aligning = false, commandedSpeed = 0.0f))
        assertFalse(PhysBearingFollowController.shouldForceMovingFollowUpdate("FOLLOW_ANGLE", aligning = true, commandedSpeed = 1.0f))
        assertFalse(PhysBearingFollowController.shouldForceMovingFollowUpdate("LOCKED", aligning = false, commandedSpeed = 1.0f))
        assertFalse(PhysBearingFollowController.isFollowCommandActive("FOLLOW_ANGLE", aligning = false, commandedSpeed = 5.0e-4f, speedEps = 1.0e-3f))
        assertTrue(PhysBearingFollowController.isFollowCommandActive("FOLLOW_ANGLE", aligning = false, commandedSpeed = 1.0e-3f, speedEps = 1.0e-3f))
    }

    @Test
    fun stop_latch_holds_measured_angle_on_command_drop() {
        assertTrue(
            PhysBearingFollowController.shouldLatchFollowStop(
                modeName = "FOLLOW_ANGLE",
                aligning = false,
                wasMovingFollow = true,
                followCommandActive = false
            )
        )
        val measuredDeg = -10.0
        val normalized = PhysBearingFollowController.normalizeDisplayAngleDeg720(measuredDeg)
        assertTrue(normalized in 0f..720f)
        assertTrue(abs(normalized - 710f) < 1.0e-6f)
    }

    @Test
    fun stop_latch_clears_on_command_resume() {
        assertFalse(
            PhysBearingFollowController.shouldLatchFollowStop(
                modeName = "FOLLOW_ANGLE",
                aligning = false,
                wasMovingFollow = true,
                followCommandActive = true
            )
        )
    }

    @Test
    fun moving_authority_mass_ratio_scaling_stronger_for_heavy_main() {
        val lowRatio = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        val highRatio = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 2_000_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        assertTrue(lowRatio.maxForce <= highRatio.maxForce * 0.2)
        assertTrue(lowRatio.maxTorque <= highRatio.maxTorque * 0.35)
    }

    @Test
    fun moving_follow_authority_still_below_locked_hold() {
        val followMoving = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "FOLLOW_ANGLE",
            aligning = false,
            movingFollow = true,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.45,
            postLoadAuthorityMultiplier = 1.0
        )
        val locked = PhysBearingFollowController.computeFixedAuthorityProfile(
            modeName = "LOCKED",
            aligning = false,
            movingFollow = false,
            inPostLoadSettle = false,
            subMass = 10_000.0,
            mainMass = 2_000_000.0,
            commandedStepMagnitudeRad = 0.0,
            postLoadAuthorityMultiplier = 1.0
        )
        assertTrue(followMoving.maxForce < locked.maxForce)
        assertTrue(followMoving.maxTorque < locked.maxTorque)
    }

    @Test
    fun adaptive_catchup_monotonic_and_bounded() {
        val minGain = 0.04
        val maxGain = 0.18
        val fullError = 0.3
        val tiny = PhysBearingFollowController.computeAdaptiveCatchupGain(0.0, minGain, maxGain, fullError)
        val mid = PhysBearingFollowController.computeAdaptiveCatchupGain(0.15, minGain, maxGain, fullError)
        val large = PhysBearingFollowController.computeAdaptiveCatchupGain(2.0, minGain, maxGain, fullError)
        assertTrue(tiny in minGain..maxGain)
        assertTrue(mid in minGain..maxGain)
        assertTrue(large in minGain..maxGain)
        assertTrue(tiny <= mid)
        assertTrue(mid <= large)
        assertTrue(abs(large - maxGain) < 1.0e-9)
    }
}
