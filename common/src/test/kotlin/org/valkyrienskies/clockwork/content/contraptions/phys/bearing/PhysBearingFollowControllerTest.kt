package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class PhysBearingFollowControllerTest {
    @Test
    fun lockedModeSelectsFixedJointKind() {
        assertTrue(PhysBearingFollowController.isLockedFixedMode(isLockedMode = true, aligning = false))
        assertFalse(PhysBearingFollowController.isLockedFixedMode(isLockedMode = true, aligning = true))
    }

    @Test
    fun nonLockedModesSelectRevoluteKind() {
        assertFalse(PhysBearingFollowController.isLockedFixedMode(isLockedMode = false, aligning = false))
        assertFalse(PhysBearingFollowController.isLockedFixedMode(isLockedMode = false, aligning = true))
    }

    @Test
    fun snapDeltaUsesShortestPath() {
        val nearWrapPositive = PhysBearingFollowController.normalizedSnapDeltaRad(
            targetAngleRad = Math.toRadians(350.0),
            currentAngleRad = Math.toRadians(10.0)
        )
        val nearWrapNegative = PhysBearingFollowController.normalizedSnapDeltaRad(
            targetAngleRad = Math.toRadians(10.0),
            currentAngleRad = Math.toRadians(350.0)
        )
        assertTrue(nearWrapPositive < 0.0)
        assertTrue(nearWrapNegative > 0.0)
        assertTrue(kotlin.math.abs(nearWrapPositive) < Math.PI)
        assertTrue(kotlin.math.abs(nearWrapNegative) < Math.PI)
    }

    @Test
    fun brakeDecelTransitionsIntoHoldWithCaptureHysteresis() {
        var previousWasTrack = false
        var holdLatched: Double? = null
        var captureTicks = 0
        var emergencyTicks = 0
        var last = PhysBearingServoMath.FollowPhase.BRAKE_DECEL

        repeat(2) {
            val out = PhysBearingFollowController.evolvePhase(
                PhysBearingFollowController.PhaseRequest(
                    followAngleStalled = false,
                    commandOmegaRadSec = 0.0,
                    omegaActualRadSec = 0.03,
                    previousWasTrack = previousWasTrack,
                    holdLatchedAngle = holdLatched,
                    captureTicks = captureTicks,
                    emergencyTicks = emergencyTicks,
                    captureOmegaRadSec = 0.05,
                    captureTicksRequired = 3,
                    emergencyReleaseOmegaRadSec = 1.8,
                    emergencyReleaseTicksRequired = 4,
                    commandActiveEpsilonRadSec = 1.0e-3
                )
            )
            last = out.phase
            previousWasTrack = out.trackPhase
            holdLatched = if (out.holdLatched) 1.0 else null
            captureTicks = out.captureTicks
            emergencyTicks = out.emergencyTicks
            assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_DECEL, out.phase)
        }

        val enterHold = PhysBearingFollowController.evolvePhase(
            PhysBearingFollowController.PhaseRequest(
                followAngleStalled = false,
                commandOmegaRadSec = 0.0,
                omegaActualRadSec = 0.03,
                previousWasTrack = previousWasTrack,
                holdLatchedAngle = holdLatched,
                captureTicks = captureTicks,
                emergencyTicks = emergencyTicks,
                captureOmegaRadSec = 0.05,
                captureTicksRequired = 3,
                emergencyReleaseOmegaRadSec = 1.8,
                emergencyReleaseTicksRequired = 4,
                commandActiveEpsilonRadSec = 1.0e-3
            )
        )
        assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_HOLD, enterHold.phase)
        assertTrue(enterHold.holdPhase)
        assertTrue(enterHold.enteredHoldFromDecel)

        last = enterHold.phase
        previousWasTrack = enterHold.trackPhase
        holdLatched = if (enterHold.holdLatched) 1.0 else null
        captureTicks = enterHold.captureTicks
        emergencyTicks = enterHold.emergencyTicks

        repeat(3) {
            val out = PhysBearingFollowController.evolvePhase(
                PhysBearingFollowController.PhaseRequest(
                    followAngleStalled = false,
                    commandOmegaRadSec = 0.0,
                    omegaActualRadSec = 0.08,
                    previousWasTrack = previousWasTrack,
                    holdLatchedAngle = holdLatched,
                    captureTicks = captureTicks,
                    emergencyTicks = emergencyTicks,
                    captureOmegaRadSec = 0.05,
                    captureTicksRequired = 3,
                    emergencyReleaseOmegaRadSec = 1.8,
                    emergencyReleaseTicksRequired = 4,
                    commandActiveEpsilonRadSec = 1.0e-3
                )
            )
            last = out.phase
            previousWasTrack = out.trackPhase
            holdLatched = if (out.holdLatched) 1.0 else null
            captureTicks = out.captureTicks
            emergencyTicks = out.emergencyTicks
            assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_HOLD, out.phase)
        }
        assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_HOLD, last)

        repeat(4) {
            val out = PhysBearingFollowController.evolvePhase(
                PhysBearingFollowController.PhaseRequest(
                    followAngleStalled = false,
                    commandOmegaRadSec = 0.0,
                    omegaActualRadSec = 2.0,
                    previousWasTrack = previousWasTrack,
                    holdLatchedAngle = holdLatched,
                    captureTicks = captureTicks,
                    emergencyTicks = emergencyTicks,
                    captureOmegaRadSec = 0.05,
                    captureTicksRequired = 3,
                    emergencyReleaseOmegaRadSec = 1.8,
                    emergencyReleaseTicksRequired = 4,
                    commandActiveEpsilonRadSec = 1.0e-3
                )
            )
            last = out.phase
            previousWasTrack = out.trackPhase
            holdLatched = if (out.holdLatched) 1.0 else null
            captureTicks = out.captureTicks
            emergencyTicks = out.emergencyTicks
        }
        assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_DECEL, last)
    }

    @Test
    fun holdSupportModeDoesNotAllowDropoutInsideFollowHold() {
        val context = PhysBearingStabilizerController.HoldContext(
            followMode = true,
            followBrakePhase = false,
            followHoldPhase = true,
            followRestUltraStable = true,
            followUltraSleep = true,
            followRigidRest = true,
            followRigidRestBlend = 1.0,
            followHoldSettleActive = false,
            followAcquireActive = false
        )

        val withBias = PhysBearingStabilizerController.resolveHoldSupportMode(
            context = context,
            biasActive = true
        )
        val withoutBias = PhysBearingStabilizerController.resolveHoldSupportMode(
            context = context,
            biasActive = false
        )

        assertFalse(withBias.allowNearZeroDropout)
        assertFalse(withoutBias.allowNearZeroDropout)
        assertTrue(withBias.dampingScale >= 1.0)
        assertTrue(withoutBias.dampingScale >= 1.0)
    }
}
