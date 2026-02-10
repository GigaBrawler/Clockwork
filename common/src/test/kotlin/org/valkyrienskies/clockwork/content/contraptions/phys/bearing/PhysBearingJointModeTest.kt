package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import kotlin.math.PI
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class PhysBearingJointModeTest {
    @Test
    fun followAndLockedUseFixedJointMode() {
        assertTrue(PhysBearingFollowController.isFixedJointMode("FOLLOW_ANGLE", aligning = false))
        assertTrue(PhysBearingFollowController.isFixedJointMode("LOCKED", aligning = false))
        assertFalse(PhysBearingFollowController.isFixedJointMode("UNLOCKED", aligning = false))
    }

    @Test
    fun aligningForcesFixedModeForAnySelection() {
        listOf("UNLOCKED", "FOLLOW_ANGLE", "LOCKED").forEach { modeName ->
            assertTrue(PhysBearingFollowController.isFixedJointMode(modeName, aligning = true))
        }
    }

    @Test
    fun normalizedSnapDeltaStaysWithinPi() {
        val delta = PhysBearingFollowController.normalizedSnapDeltaRad(
            targetAngleRad = Math.toRadians(721.0),
            currentAngleRad = Math.toRadians(-720.0)
        )
        assertTrue(delta in -PI..PI)
    }

    @Test
    fun jointKindMismatchRequestsRematerializationUpdate() {
        assertTrue(
            PhysBearingFollowController.shouldApplyFixedTargetUpdate(
                modeOrAlignmentTransition = false,
                jointKindMismatch = true,
                targetDeltaAbsRad = 0.0,
                driftAbsRad = 0.0,
                targetEpsRad = 1.0e-3,
                holdDriftDeadbandRad = 1.0e-3,
                ticksSinceLastRefresh = 0,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun modeTransitionRequestsRematerializationUpdate() {
        assertTrue(
            PhysBearingFollowController.shouldApplyFixedTargetUpdate(
                modeOrAlignmentTransition = true,
                jointKindMismatch = false,
                targetDeltaAbsRad = 0.0,
                driftAbsRad = 0.0,
                targetEpsRad = 1.0e-3,
                holdDriftDeadbandRad = 1.0e-3,
                ticksSinceLastRefresh = 0,
                safetyRefreshTicks = 200
            )
        )
    }

    @Test
    fun fullTurnEquivalentTargetDoesNotCreateLargeDelta() {
        val currentActive = 6.0 * PI + 0.02
        val wrappedTarget = 0.03
        val desiredUnwrapped = PhysBearingFollowController.unwrapAngleNearReferenceRad(currentActive, wrappedTarget)
        val delta = PhysBearingFollowController.normalizeAngleErrorRad(desiredUnwrapped, currentActive)
        assertTrue(delta > 0.0)
        assertTrue(abs(delta) < 0.05)
    }

    @Test
    fun fixedTargetUpdateWrapCrossingRemainsSmallAndFinite() {
        val lastSent = 2.0 * PI - 0.01
        val wrappedTarget = 0.0
        val desiredUnwrapped = PhysBearingFollowController.unwrapAngleNearReferenceRad(lastSent, wrappedTarget)
        val stepped = PhysBearingFollowController.stepTowardAngleRad(lastSent, desiredUnwrapped, 0.1)
        val delta = abs(PhysBearingFollowController.normalizeAngleErrorRad(stepped, lastSent))
        assertTrue(stepped.isFinite())
        assertTrue(delta.isFinite())
        assertTrue(delta <= 0.02)
    }

    @Test
    fun wrap_crossing_with_lagged_active_remains_small_and_finite() {
        val previousDesired = 8.0 * PI - 0.01
        val laggedMeasured = 6.0 * PI + 0.9
        val wrappedTarget = 0.01
        val desired = PhysBearingFollowController.selectDesiredContinuousTarget(
            wrappedTargetRad = wrappedTarget,
            measuredAngleRad = laggedMeasured,
            previousDesiredContinuousRad = previousDesired,
            inFollowSettleWindow = false
        )
        val delta = abs(PhysBearingFollowController.normalizeAngleErrorRad(desired, previousDesired))
        assertTrue(desired.isFinite())
        assertTrue(delta.isFinite())
        assertTrue(delta < 0.05)
    }

    @Test
    fun reload_settle_prevents_immediate_large_target_jump() {
        val measured = 5.2
        val wrappedTarget = 0.1
        val selected = PhysBearingFollowController.selectDesiredContinuousTarget(
            wrappedTargetRad = wrappedTarget,
            measuredAngleRad = measured,
            previousDesiredContinuousRad = 12.5,
            inFollowSettleWindow = true
        )
        assertTrue(selected.isFinite())
        assertTrue(abs(selected - measured) < 1.0e-12)
    }
}
