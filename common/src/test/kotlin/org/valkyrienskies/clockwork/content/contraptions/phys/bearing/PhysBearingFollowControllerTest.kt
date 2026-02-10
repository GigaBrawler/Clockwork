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
    fun movingFollowRequestsTickUpdates() {
        assertTrue(PhysBearingFollowController.shouldForceMovingFollowUpdate("FOLLOW_ANGLE", aligning = false, commandedSpeed = 1.0f))
        assertFalse(PhysBearingFollowController.shouldForceMovingFollowUpdate("FOLLOW_ANGLE", aligning = false, commandedSpeed = 0.0f))
        assertFalse(PhysBearingFollowController.shouldForceMovingFollowUpdate("FOLLOW_ANGLE", aligning = true, commandedSpeed = 1.0f))
        assertFalse(PhysBearingFollowController.shouldForceMovingFollowUpdate("LOCKED", aligning = false, commandedSpeed = 1.0f))
    }
}
