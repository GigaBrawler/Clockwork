package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sqrt

class PhysBearingServoMathTest {
    @Test
    fun brakeTorqueIsAlwaysDissipative() {
        val params = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)
        val inertia = 250.0
        val stopOmega = 0.05

        for (omega in listOf(-20.0, -5.0, -0.5, 0.5, 5.0, 20.0)) {
            val alpha = PhysBearingServoMath.computeFrictionBrakeAlpha(
                omegaActualRadSec = omega,
                brakeRate = params.brakeRate,
                staticBrakeAlpha = params.brakeStaticAlpha,
                stopOmegaRadSec = stopOmega
            )
            val tau = inertia * alpha
            assertTrue(tau * omega <= 1.0e-9)
        }
    }

    @Test
    fun noReverseClampPreventsSignFlip() {
        val dt = 1.0 / 20.0
        for (omega in listOf(-40.0, -5.0, -0.2, 0.2, 5.0, 40.0)) {
            val hugeOpposingAlpha = -sign(omega) * 1.0e6
            val clamped = PhysBearingServoMath.clampNoReverse(hugeOpposingAlpha, omega, dt)
            val nextOmega = omega + clamped * dt
            assertTrue(nextOmega == 0.0 || sign(nextOmega) == sign(omega))
            assertTrue(abs(nextOmega) <= abs(omega) + 1.0e-9)
        }
    }

    @Test
    fun snapToZeroNoReverse() {
        val dt = 1.0 / 20.0
        for (omega in listOf(-2.0, -0.4, -0.08, 0.08, 0.4, 2.0)) {
            val snapAlpha = PhysBearingServoMath.computeSnapToZeroAlpha(omega, dt)
            val clamped = PhysBearingServoMath.clampNoReverse(snapAlpha, omega, dt)
            val nextOmega = omega + clamped * dt
            assertTrue(abs(nextOmega) <= 1.0e-9)
        }
    }

    @Test
    fun strengthMappingIsMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.stopTimeSec > mid.stopTimeSec && mid.stopTimeSec > high.stopTimeSec)
        assertTrue(low.brakeRate < mid.brakeRate && mid.brakeRate < high.brakeRate)
        assertTrue(low.maxBrakeAlpha < mid.maxBrakeAlpha && mid.maxBrakeAlpha < high.maxBrakeAlpha)
        assertTrue(low.trackOmegaGain < mid.trackOmegaGain && mid.trackOmegaGain < high.trackOmegaGain)
        assertTrue(low.trackPosAssistGain < mid.trackPosAssistGain && mid.trackPosAssistGain < high.trackPosAssistGain)
        assertTrue(
            low.trackPosAssistOmegaLimit < mid.trackPosAssistOmegaLimit &&
                mid.trackPosAssistOmegaLimit < high.trackPosAssistOmegaLimit
        )
    }

    @Test
    fun holdDampingFloorRespected() {
        for (strength in listOf(0.0, 0.5, 1.0)) {
            val p = PhysBearingServoMath.mapFollowStrength(strength01 = strength, sliderScale = 1.0)
            val kdEff = PhysBearingServoMath.holdKdWithDampingFloor(
                holdKpAlpha = p.holdKpAlpha,
                holdKdAlphaMapped = p.holdKdAlpha,
                holdDampingZetaMin = p.holdDampingZetaMin
            )
            val kdFloor = 2.0 * p.holdDampingZetaMin * sqrt(max(p.holdKpAlpha, 1.0e-9))
            assertTrue(kdEff + 1.0e-9 >= kdFloor)
        }
    }

    @Test
    fun holdPdRestoringAndDissipative() {
        val p = PhysBearingServoMath.mapFollowStrength(strength01 = 0.75, sliderScale = 1.0)
        val deadbandError = 0.0015
        val deadbandOmega = 0.015

        val alphaPositiveError = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = 0.10,
            omegaActualRadSec = 0.0,
            holdKpAlpha = p.holdKpAlpha,
            holdKdAlphaMapped = p.holdKdAlpha,
            holdDampingZetaMin = p.holdDampingZetaMin,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaNegativeError = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = -0.10,
            omegaActualRadSec = 0.0,
            holdKpAlpha = p.holdKpAlpha,
            holdKdAlphaMapped = p.holdKdAlpha,
            holdDampingZetaMin = p.holdDampingZetaMin,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaPositiveOmega = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = 0.0,
            omegaActualRadSec = 0.20,
            holdKpAlpha = p.holdKpAlpha,
            holdKdAlphaMapped = p.holdKdAlpha,
            holdDampingZetaMin = p.holdDampingZetaMin,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaNegativeOmega = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = 0.0,
            omegaActualRadSec = -0.20,
            holdKpAlpha = p.holdKpAlpha,
            holdKdAlphaMapped = p.holdKdAlpha,
            holdDampingZetaMin = p.holdDampingZetaMin,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )

        assertTrue(alphaPositiveError > 0.0)
        assertTrue(alphaNegativeError < 0.0)
        assertTrue(alphaPositiveOmega < 0.0)
        assertTrue(alphaNegativeOmega > 0.0)
    }

    @Test
    fun phaseCaptureHysteresisNoThrash() {
        val captureOmega = 0.08
        val captureTicks = 3
        val emergencyOmega = 1.8
        val emergencyTicks = 4
        var state = PhysBearingServoMath.FollowPhaseState(
            phase = PhysBearingServoMath.FollowPhase.BRAKE_DECEL,
            holdLatched = false,
            captureTicks = 0,
            emergencyTicks = 0
        )

        repeat(captureTicks - 1) {
            state = PhysBearingServoMath.stepFollowPhase(
                trackActive = false,
                omegaAbsRadSec = captureOmega * 0.75,
                previous = state,
                captureOmegaRadSec = captureOmega,
                captureTicksRequired = captureTicks,
                emergencyReleaseOmegaRadSec = emergencyOmega,
                emergencyReleaseTicksRequired = emergencyTicks
            )
            assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_DECEL, state.phase)
        }
        state = PhysBearingServoMath.stepFollowPhase(
            trackActive = false,
            omegaAbsRadSec = captureOmega * 0.75,
            previous = state,
            captureOmegaRadSec = captureOmega,
            captureTicksRequired = captureTicks,
            emergencyReleaseOmegaRadSec = emergencyOmega,
            emergencyReleaseTicksRequired = emergencyTicks
        )
        assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_HOLD, state.phase)
        assertTrue(state.holdLatched)

        for (omega in listOf(0.02, 0.14, 0.04, 0.11, 0.06, 0.13, 0.05)) {
            state = PhysBearingServoMath.stepFollowPhase(
                trackActive = false,
                omegaAbsRadSec = omega,
                previous = state,
                captureOmegaRadSec = captureOmega,
                captureTicksRequired = captureTicks,
                emergencyReleaseOmegaRadSec = emergencyOmega,
                emergencyReleaseTicksRequired = emergencyTicks
            )
            assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_HOLD, state.phase)
            assertTrue(state.holdLatched)
        }

        repeat(emergencyTicks - 1) {
            state = PhysBearingServoMath.stepFollowPhase(
                trackActive = false,
                omegaAbsRadSec = emergencyOmega + 0.2,
                previous = state,
                captureOmegaRadSec = captureOmega,
                captureTicksRequired = captureTicks,
                emergencyReleaseOmegaRadSec = emergencyOmega,
                emergencyReleaseTicksRequired = emergencyTicks
            )
            assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_HOLD, state.phase)
        }
        state = PhysBearingServoMath.stepFollowPhase(
            trackActive = false,
            omegaAbsRadSec = emergencyOmega + 0.2,
            previous = state,
            captureOmegaRadSec = captureOmega,
            captureTicksRequired = captureTicks,
            emergencyReleaseOmegaRadSec = emergencyOmega,
            emergencyReleaseTicksRequired = emergencyTicks
        )
        assertEquals(PhysBearingServoMath.FollowPhase.BRAKE_DECEL, state.phase)
        assertFalse(state.holdLatched)
    }

    @Test
    fun ringdownBoostActivatesOnFlipSequence() {
        val durationTicks = 8
        var ringTicks = 0
        var errorSign = 0
        val boost = 1.6

        run {
            val state = PhysBearingServoMath.updateHoldRingDown(
                currentTicks = ringTicks,
                previousErrorSign = errorSign,
                holdErrorRad = 0.03,
                omegaActualRadSec = 0.20,
                errorMinRad = 0.005,
                omegaMinRadSec = 0.08,
                durationTicks = durationTicks
            )
            ringTicks = state.ticks
            errorSign = state.errorSign
        }
        assertEquals(0, ringTicks)

        run {
            val state = PhysBearingServoMath.updateHoldRingDown(
                currentTicks = ringTicks,
                previousErrorSign = errorSign,
                holdErrorRad = -0.03,
                omegaActualRadSec = 0.20,
                errorMinRad = 0.005,
                omegaMinRadSec = 0.08,
                durationTicks = durationTicks
            )
            ringTicks = state.ticks
            errorSign = state.errorSign
        }
        assertEquals(durationTicks, ringTicks)
        assertTrue(PhysBearingServoMath.holdRingDownKdMultiplier(ringTicks, boost) > 1.0)

        repeat(durationTicks) {
            val state = PhysBearingServoMath.updateHoldRingDown(
                currentTicks = ringTicks,
                previousErrorSign = errorSign,
                holdErrorRad = 0.0,
                omegaActualRadSec = 0.0,
                errorMinRad = 0.005,
                omegaMinRadSec = 0.08,
                durationTicks = durationTicks
            )
            ringTicks = state.ticks
            errorSign = state.errorSign
        }
        assertEquals(0, ringTicks)
        assertEquals(1.0, PhysBearingServoMath.holdRingDownKdMultiplier(ringTicks, boost))
    }

    @Test
    fun strengthMonotonicRigidHold() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.holdKpAlpha < mid.holdKpAlpha && mid.holdKpAlpha < high.holdKpAlpha)
        assertTrue(low.holdKdAlpha < mid.holdKdAlpha && mid.holdKdAlpha < high.holdKdAlpha)
        assertTrue(low.holdDampingZetaMin <= mid.holdDampingZetaMin && mid.holdDampingZetaMin <= high.holdDampingZetaMin)
        assertTrue(low.holdMaxAlpha < mid.holdMaxAlpha && mid.holdMaxAlpha < high.holdMaxAlpha)
        assertTrue(
            low.holdRingDownKdBoost <= mid.holdRingDownKdBoost &&
                mid.holdRingDownKdBoost <= high.holdRingDownKdBoost
        )

        val kdLow = PhysBearingServoMath.holdKdWithDampingFloor(low.holdKpAlpha, low.holdKdAlpha, low.holdDampingZetaMin)
        val kdMid = PhysBearingServoMath.holdKdWithDampingFloor(mid.holdKpAlpha, mid.holdKdAlpha, mid.holdDampingZetaMin)
        val kdHigh = PhysBearingServoMath.holdKdWithDampingFloor(high.holdKpAlpha, high.holdKdAlpha, high.holdDampingZetaMin)
        assertTrue(kdLow <= kdMid && kdMid <= kdHigh)
    }

    @Test
    fun smallNonzeroCommandIsActive() {
        val eps = 1.0e-3
        assertFalse(PhysBearingServoMath.isFollowCommandActive(0.0, eps))
        assertFalse(PhysBearingServoMath.isFollowCommandActive(5.0e-4, eps))
        assertTrue(PhysBearingServoMath.isFollowCommandActive(1.0e-3, eps))
        assertTrue(PhysBearingServoMath.isFollowCommandActive(2.5e-3, eps))
    }

    @Test
    fun stallThresholdBoundedNoFalseHighCmd() {
        val highCmdDelta = 255.0 * Math.PI / 180.0
        val eps = PhysBearingServoMath.boundedStallEpsilonRad(
            cmdDeltaRad = highCmdDelta,
            baseEpsRad = 1.0e-4,
            commandFraction = 4.0e-4,
            maxEpsRad = 0.004
        )
        assertTrue(eps in 1.0e-4..0.004)
        assertTrue(eps < 0.01)
    }

    @Test
    fun stallUnlatchOnMotionOrCommandStep() {
        var state = PhysBearingServoMath.FollowStallState(stalled = false, stallTicks = 0, clearTicks = 0, graceTicks = 0)
        repeat(14) {
            state = PhysBearingServoMath.stepFollowStallState(
                previous = state,
                stallCountEligible = true,
                clearMotionEligible = false,
                commandStep = false,
                stallTicksRequired = 8,
                clearTicksRequired = 3,
                graceTicksOnCommandStep = 6
            )
        }
        assertTrue(state.stalled)

        state = PhysBearingServoMath.stepFollowStallState(
            previous = state,
            stallCountEligible = false,
            clearMotionEligible = false,
            commandStep = true,
            stallTicksRequired = 8,
            clearTicksRequired = 3,
            graceTicksOnCommandStep = 6
        )
        assertFalse(state.stalled)

        repeat(14) {
            state = PhysBearingServoMath.stepFollowStallState(
                previous = state,
                stallCountEligible = true,
                clearMotionEligible = false,
                commandStep = false,
                stallTicksRequired = 8,
                clearTicksRequired = 3,
                graceTicksOnCommandStep = 6
            )
        }
        assertTrue(state.stalled)
        repeat(3) {
            state = PhysBearingServoMath.stepFollowStallState(
                previous = state,
                stallCountEligible = false,
                clearMotionEligible = true,
                commandStep = false,
                stallTicksRequired = 8,
                clearTicksRequired = 3,
                graceTicksOnCommandStep = 6
            )
        }
        assertFalse(state.stalled)
    }

    @Test
    fun trackAuthorityNotOmegaThrottled() {
        val low = PhysBearingServoMath.computeTrackAuthority(
            chainFactor = 0.55,
            offAxisFactor = 0.80,
            relVelFactor = 0.90,
            minAuthority = 0.18
        )
        val high = PhysBearingServoMath.computeTrackAuthority(
            chainFactor = 0.55,
            offAxisFactor = 0.80,
            relVelFactor = 0.90,
            minAuthority = 0.18
        )
        assertEquals(low, high, 1.0e-12)
    }

    @Test
    fun rigidRestHysteresisNoChatter() {
        var state = PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        val enterTicks = 4

        val jitterSequence = listOf(true, false, true, false, true, false, true, false)
        for (enter in jitterSequence) {
            state = PhysBearingServoMath.stepHysteresisLatch(
                previouslyActive = state.active,
                previousTicks = state.ticks,
                eligible = true,
                enterCondition = enter,
                exitCondition = false,
                enterTicksRequired = enterTicks
            )
            assertFalse(state.active)
        }
    }

    @Test
    fun rigidRestExitOnDisturbance() {
        var state = PhysBearingServoMath.HysteresisLatchState(active = true, ticks = 5)
        state = PhysBearingServoMath.stepHysteresisLatch(
            previouslyActive = state.active,
            previousTicks = state.ticks,
            eligible = true,
            enterCondition = true,
            exitCondition = false,
            enterTicksRequired = 4
        )
        assertTrue(state.active)

        state = PhysBearingServoMath.stepHysteresisLatch(
            previouslyActive = state.active,
            previousTicks = state.ticks,
            eligible = true,
            enterCondition = false,
            exitCondition = true,
            enterTicksRequired = 4
        )
        assertFalse(state.active)
    }

    @Test
    fun trackCommandFidelityNoChainSpeedShaping() {
        val command = 18.0
        val targetNoAssist = PhysBearingServoMath.computeTrackOmegaTarget(
            commandOmegaRadSec = command,
            errorRad = 0.0,
            posAssistGain = 10.0,
            posAssistOmegaLimit = 100.0,
            maxOmegaRadSec = 80.0
        )
        val hypotheticalAuthorityScaled = command * 0.25
        assertEquals(command, targetNoAssist, 1.0e-9)
        assertTrue(abs(targetNoAssist - hypotheticalAuthorityScaled) > 1.0e-6)
    }

    @Test
    fun strength100RestFloorHighest() {
        val low = PhysBearingServoMath.holdRestAuthorityFloor(0.0, rigidBlend01 = 0.0)
        val mid = PhysBearingServoMath.holdRestAuthorityFloor(0.5, rigidBlend01 = 0.0)
        val high = PhysBearingServoMath.holdRestAuthorityFloor(1.0, rigidBlend01 = 0.0)
        val highRigid = PhysBearingServoMath.holdRestAuthorityFloor(1.0, rigidBlend01 = 1.0)
        assertTrue(low <= mid && mid <= high)
        assertTrue(high <= highRigid)
    }

    @Test
    fun rigidHingeMicroHysteresisNoChatter() {
        var state = PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        val jitter = listOf(true, false, true, false, true, false, true, false)
        for (enter in jitter) {
            state = PhysBearingServoMath.stepHysteresisLatch(
                previouslyActive = state.active,
                previousTicks = state.ticks,
                eligible = true,
                enterCondition = enter,
                exitCondition = false,
                enterTicksRequired = 3
            )
        }
        assertFalse(state.active)
    }

    @Test
    fun holdMicroNeverZeroOffaxisStiffness() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)
        assertTrue(low.holdRestTiltStiffnessFloor > 0.0)
        assertTrue(mid.holdRestTiltStiffnessFloor > 0.0)
        assertTrue(high.holdRestTiltStiffnessFloor > 0.0)
    }

    @Test
    fun strengthMonotonicFollowCapsAndFloors() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.trackAuthorityFloor <= mid.trackAuthorityFloor && mid.trackAuthorityFloor <= high.trackAuthorityFloor)
        assertTrue(
            low.trackMaxOmegaStepPerTick <= mid.trackMaxOmegaStepPerTick &&
                mid.trackMaxOmegaStepPerTick <= high.trackMaxOmegaStepPerTick
        )
        assertTrue(low.holdWorldSeatKpBoost <= mid.holdWorldSeatKpBoost && mid.holdWorldSeatKpBoost <= high.holdWorldSeatKpBoost)
        assertTrue(low.holdWorldSeatKdBoost <= mid.holdWorldSeatKdBoost && mid.holdWorldSeatKdBoost <= high.holdWorldSeatKdBoost)
        assertTrue(low.holdWorldTiltKpBoost <= mid.holdWorldTiltKpBoost && mid.holdWorldTiltKpBoost <= high.holdWorldTiltKpBoost)
        assertTrue(low.holdWorldTiltKdBoost <= mid.holdWorldTiltKdBoost && mid.holdWorldTiltKdBoost <= high.holdWorldTiltKdBoost)
        assertTrue(
            low.holdWorldTiltAlphaCapScale <= mid.holdWorldTiltAlphaCapScale &&
                mid.holdWorldTiltAlphaCapScale <= high.holdWorldTiltAlphaCapScale
        )
        assertTrue(
            low.holdWorldTiltAlphaEqCapScale <= mid.holdWorldTiltAlphaEqCapScale &&
                mid.holdWorldTiltAlphaEqCapScale <= high.holdWorldTiltAlphaEqCapScale
        )
        assertTrue(
            low.holdRestTiltStiffnessFloor <= mid.holdRestTiltStiffnessFloor &&
                mid.holdRestTiltStiffnessFloor <= high.holdRestTiltStiffnessFloor
        )
    }

    @Test
    fun sameDirectionAssistGate() {
        assertTrue(PhysBearingServoMath.sameDirectionAssistEnabled(0.2, 4.0))
        assertTrue(PhysBearingServoMath.sameDirectionAssistEnabled(-0.2, -4.0))
        assertFalse(PhysBearingServoMath.sameDirectionAssistEnabled(-0.2, 4.0))
        assertFalse(PhysBearingServoMath.sameDirectionAssistEnabled(0.2, -4.0))
        assertFalse(PhysBearingServoMath.sameDirectionAssistEnabled(0.0, 2.0))
        assertFalse(PhysBearingServoMath.sameDirectionAssistEnabled(0.1, 0.0))
    }

    @Test
    fun chainAttenuationIsBoundedAndMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.chainTorqueScale in 0.0..1.0)
        assertTrue(mid.chainTorqueScale in 0.0..1.0)
        assertTrue(high.chainTorqueScale in 0.0..1.0)
        assertTrue(low.chainStiffnessScale in 0.0..1.0)
        assertTrue(mid.chainStiffnessScale in 0.0..1.0)
        assertTrue(high.chainStiffnessScale in 0.0..1.0)
        assertTrue(low.chainTorqueScale <= mid.chainTorqueScale && mid.chainTorqueScale <= high.chainTorqueScale)
        assertTrue(low.chainStiffnessScale <= mid.chainStiffnessScale && mid.chainStiffnessScale <= high.chainStiffnessScale)
    }

    @Test
    fun followBrakeStiffnessFloorNonzero() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.brakeTiltStiffnessFloor > 0.0 && low.brakeTiltStiffnessFloor <= 1.0)
        assertTrue(mid.brakeTiltStiffnessFloor > 0.0 && mid.brakeTiltStiffnessFloor <= 1.0)
        assertTrue(high.brakeTiltStiffnessFloor > 0.0 && high.brakeTiltStiffnessFloor <= 1.0)
        assertTrue(low.brakeTiltStiffnessFloor <= mid.brakeTiltStiffnessFloor)
        assertTrue(mid.brakeTiltStiffnessFloor <= high.brakeTiltStiffnessFloor)
    }
}
