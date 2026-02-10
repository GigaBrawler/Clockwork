package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.joml.Vector3d
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
        assertTrue(
            low.fixedHoldSeatAuthorityFloor <= mid.fixedHoldSeatAuthorityFloor &&
                mid.fixedHoldSeatAuthorityFloor <= high.fixedHoldSeatAuthorityFloor
        )
        assertTrue(
            low.fixedHoldTiltAuthorityFloor <= mid.fixedHoldTiltAuthorityFloor &&
                mid.fixedHoldTiltAuthorityFloor <= high.fixedHoldTiltAuthorityFloor
        )
        assertTrue(
            low.fixedHoldSeatKpFloorScale <= mid.fixedHoldSeatKpFloorScale &&
                mid.fixedHoldSeatKpFloorScale <= high.fixedHoldSeatKpFloorScale
        )
        assertTrue(
            low.fixedHoldTiltStiffnessFloor <= mid.fixedHoldTiltStiffnessFloor &&
                mid.fixedHoldTiltStiffnessFloor <= high.fixedHoldTiltStiffnessFloor
        )
        assertTrue(
            low.fixedOffAxisDampingZetaMin <= mid.fixedOffAxisDampingZetaMin &&
                mid.fixedOffAxisDampingZetaMin <= high.fixedOffAxisDampingZetaMin
        )
    }

    @Test
    fun fixedLikeHoldFloorsMonotonicStrength() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.fixedHoldSeatAuthorityFloor <= mid.fixedHoldSeatAuthorityFloor)
        assertTrue(mid.fixedHoldSeatAuthorityFloor <= high.fixedHoldSeatAuthorityFloor)
        assertTrue(low.fixedHoldTiltAuthorityFloor <= mid.fixedHoldTiltAuthorityFloor)
        assertTrue(mid.fixedHoldTiltAuthorityFloor <= high.fixedHoldTiltAuthorityFloor)
        assertTrue(low.fixedHoldSeatKpFloorScale <= mid.fixedHoldSeatKpFloorScale)
        assertTrue(mid.fixedHoldSeatKpFloorScale <= high.fixedHoldSeatKpFloorScale)
        assertTrue(low.fixedHoldTiltStiffnessFloor <= mid.fixedHoldTiltStiffnessFloor)
        assertTrue(mid.fixedHoldTiltStiffnessFloor <= high.fixedHoldTiltStiffnessFloor)
        assertTrue(low.fixedOffAxisDampingZetaMin <= mid.fixedOffAxisDampingZetaMin)
        assertTrue(mid.fixedOffAxisDampingZetaMin <= high.fixedOffAxisDampingZetaMin)
    }

    @Test
    fun fixedLikeHoldNeverZeroSupportWhenInHold() {
        val active = PhysBearingServoMath.computeHoldSupportMode(
            inFollowHold = true,
            ultraSleep = false,
            restUltraStable = false,
            settleActive = false,
            acquireActive = false,
            biasActive = true
        )
        assertFalse(active.allowNearZeroDropout)
        assertTrue(active.dampingScale > 0.0)
        assertTrue(active.stiffnessScale > 0.0)

        val ultra = PhysBearingServoMath.computeHoldSupportMode(
            inFollowHold = true,
            ultraSleep = true,
            restUltraStable = true,
            settleActive = false,
            acquireActive = false,
            biasActive = false
        )
        assertFalse(ultra.allowNearZeroDropout)
        assertTrue(ultra.dampingScale >= 1.0)
        assertTrue(ultra.stiffnessScale > 0.0)
    }

    @Test
    fun fixedHoldFloorProfileHelperFiniteBounded() {
        val mapped = PhysBearingServoMath.mapFollowStrength(strength01 = 0.8, sliderScale = 1.0)
        val floor = PhysBearingServoMath.applyFixedHoldFloorProfile(
            seatAuthorityFloorMapped = mapped.fixedHoldSeatAuthorityFloor,
            tiltAuthorityFloorMapped = mapped.fixedHoldTiltAuthorityFloor,
            seatKpFloorScaleMapped = mapped.fixedHoldSeatKpFloorScale,
            tiltStiffnessFloorMapped = mapped.fixedHoldTiltStiffnessFloor,
            offAxisDampingZetaMinMapped = mapped.fixedOffAxisDampingZetaMin,
            rigidBlend01 = 0.7
        )
        assertTrue(floor.seatAuthorityFloor.isFinite() && floor.seatAuthorityFloor in 0.0..1.0)
        assertTrue(floor.tiltAuthorityFloor.isFinite() && floor.tiltAuthorityFloor in 0.0..1.0)
        assertTrue(floor.seatKpFloorScale.isFinite() && floor.seatKpFloorScale in 0.0..1.0)
        assertTrue(floor.tiltStiffnessFloor.isFinite() && floor.tiltStiffnessFloor in 0.0..1.0)
        assertTrue(floor.offAxisDampingZetaMin.isFinite() && floor.offAxisDampingZetaMin >= 1.0)
    }

    @Test
    fun offaxisKdFloorOverdampedStrengthMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        val kp = 18.0
        val kdMapped = 0.8
        val kdLow = PhysBearingServoMath.kdWithFloor(kp, kdMapped, low.fixedOffAxisDampingZetaMin)
        val kdMid = PhysBearingServoMath.kdWithFloor(kp, kdMapped, mid.fixedOffAxisDampingZetaMin)
        val kdHigh = PhysBearingServoMath.kdWithFloor(kp, kdMapped, high.fixedOffAxisDampingZetaMin)
        assertTrue(kdLow <= kdMid && kdMid <= kdHigh)
    }

    @Test
    fun verticalAxisBlendMonotonic() {
        val start = 0.55
        val end = 0.90
        val b0 = PhysBearingServoMath.verticalAxisBlend(0.00, start, end)
        val b1 = PhysBearingServoMath.verticalAxisBlend(0.30, start, end)
        val b2 = PhysBearingServoMath.verticalAxisBlend(0.55, start, end)
        val b3 = PhysBearingServoMath.verticalAxisBlend(0.72, start, end)
        val b4 = PhysBearingServoMath.verticalAxisBlend(0.90, start, end)
        val b5 = PhysBearingServoMath.verticalAxisBlend(1.00, start, end)

        assertEquals(0.0, b0, 1.0e-12)
        assertEquals(0.0, b1, 1.0e-12)
        assertTrue(b0 <= b1 && b1 <= b2 && b2 <= b3 && b3 <= b4 && b4 <= b5)
        assertEquals(1.0, b4, 1.0e-12)
        assertEquals(1.0, b5, 1.0e-12)
    }

    @Test
    fun verticalHoldProfileStrengthMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(
            low.verticalHoldSeatKdScale <= mid.verticalHoldSeatKdScale &&
                mid.verticalHoldSeatKdScale <= high.verticalHoldSeatKdScale
        )
        assertTrue(
            low.verticalHoldTiltKdScale <= mid.verticalHoldTiltKdScale &&
                mid.verticalHoldTiltKdScale <= high.verticalHoldTiltKdScale
        )
        assertTrue(
            low.verticalHoldExtraOffAxisDamping <= mid.verticalHoldExtraOffAxisDamping &&
                mid.verticalHoldExtraOffAxisDamping <= high.verticalHoldExtraOffAxisDamping
        )
        assertTrue(
            low.verticalHoldSeatKpScale >= mid.verticalHoldSeatKpScale &&
                mid.verticalHoldSeatKpScale >= high.verticalHoldSeatKpScale
        )
        assertTrue(
            low.verticalHoldTiltKpScale >= mid.verticalHoldTiltKpScale &&
                mid.verticalHoldTiltKpScale >= high.verticalHoldTiltKpScale
        )
    }

    @Test
    fun verticalRestGateScaleStrengthMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.verticalHoldRestGateScale >= mid.verticalHoldRestGateScale)
        assertTrue(mid.verticalHoldRestGateScale >= high.verticalHoldRestGateScale)
        assertTrue(high.verticalHoldRestGateScale > 0.0)
    }

    @Test
    fun verticalWorldCapScalesMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(
            low.verticalHoldWorldSeatAccelCapScale <= mid.verticalHoldWorldSeatAccelCapScale &&
                mid.verticalHoldWorldSeatAccelCapScale <= high.verticalHoldWorldSeatAccelCapScale
        )
        assertTrue(
            low.verticalHoldWorldTiltAlphaCapScale <= mid.verticalHoldWorldTiltAlphaCapScale &&
                mid.verticalHoldWorldTiltAlphaCapScale <= high.verticalHoldWorldTiltAlphaCapScale
        )
        assertTrue(
            low.verticalHoldWorldTiltAlphaEqCapScale <= mid.verticalHoldWorldTiltAlphaEqCapScale &&
                mid.verticalHoldWorldTiltAlphaEqCapScale <= high.verticalHoldWorldTiltAlphaEqCapScale
        )
    }

    @Test
    fun horizontalAxisBlendMonotonic() {
        val b0 = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.00, strength01 = 1.0).horizontalBlend
        val b1 = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.25, strength01 = 1.0).horizontalBlend
        val b2 = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.55, strength01 = 1.0).horizontalBlend
        val b3 = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.80, strength01 = 1.0).horizontalBlend
        val b4 = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 1.00, strength01 = 1.0).horizontalBlend

        assertEquals(1.0, b0, 1.0e-12)
        assertEquals(0.0, b4, 1.0e-12)
        assertTrue(b0 >= b1 && b1 >= b2 && b2 >= b3 && b3 >= b4)
    }

    @Test
    fun axisHoldProfileConservativeHorizontal() {
        val low = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.0, strength01 = 0.0)
        val high = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.0, strength01 = 1.0)

        assertEquals(0.0, high.verticalBlend, 1.0e-12)
        assertEquals(1.0, high.horizontalBlend, 1.0e-12)
        assertEquals(1.00, low.seatKpScale, 1.0e-12)
        assertEquals(0.92, high.seatKpScale, 1.0e-12)
        assertEquals(1.00, low.seatKdScale, 1.0e-12)
        assertEquals(1.45, high.seatKdScale, 1.0e-12)
        assertEquals(1.00, low.tiltKpScale, 1.0e-12)
        assertEquals(0.90, high.tiltKpScale, 1.0e-12)
        assertEquals(1.00, low.tiltKdScale, 1.0e-12)
        assertEquals(1.55, high.tiltKdScale, 1.0e-12)
        assertEquals(1.00, low.restGateScale, 1.0e-12)
        assertEquals(0.65, high.restGateScale, 1.0e-12)
        assertEquals(0.00, low.extraOffAxisDamping, 1.0e-12)
        assertEquals(0.70, high.extraOffAxisDamping, 1.0e-12)
    }

    @Test
    fun axisHoldProfileVerticalPreserved() {
        val low = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 1.0, strength01 = 0.0)
        val high = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 1.0, strength01 = 1.0)

        assertEquals(1.0, high.verticalBlend, 1.0e-12)
        assertEquals(0.0, high.horizontalBlend, 1.0e-12)
        assertEquals(1.00, low.seatKpScale, 1.0e-12)
        assertEquals(0.82, high.seatKpScale, 1.0e-12)
        assertEquals(1.10, low.seatKdScale, 1.0e-12)
        assertEquals(2.20, high.seatKdScale, 1.0e-12)
        assertEquals(1.00, low.tiltKpScale, 1.0e-12)
        assertEquals(0.78, high.tiltKpScale, 1.0e-12)
        assertEquals(1.15, low.tiltKdScale, 1.0e-12)
        assertEquals(2.35, high.tiltKdScale, 1.0e-12)
        assertEquals(1.00, low.restGateScale, 1.0e-12)
        assertEquals(0.30, high.restGateScale, 1.0e-12)
        assertEquals(0.20, low.extraOffAxisDamping, 1.0e-12)
        assertEquals(2.00, high.extraOffAxisDamping, 1.0e-12)
        assertEquals(1.00, low.worldSeatAccelCapScale, 1.0e-12)
        assertEquals(2.10, high.worldSeatAccelCapScale, 1.0e-12)
        assertEquals(1.00, low.worldTiltAlphaCapScale, 1.0e-12)
        assertEquals(2.20, high.worldTiltAlphaCapScale, 1.0e-12)
        assertEquals(1.00, low.worldTiltAlphaEqCapScale, 1.0e-12)
        assertEquals(2.00, high.worldTiltAlphaEqCapScale, 1.0e-12)
    }

    @Test
    fun axisRestGateMonotonicBothAxes() {
        val horizontalLow = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.0, strength01 = 0.0)
        val horizontalMid = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.0, strength01 = 0.5)
        val horizontalHigh = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 0.0, strength01 = 1.0)
        assertTrue(horizontalLow.restGateScale >= horizontalMid.restGateScale)
        assertTrue(horizontalMid.restGateScale >= horizontalHigh.restGateScale)

        val verticalLow = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 1.0, strength01 = 0.0)
        val verticalMid = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 1.0, strength01 = 0.5)
        val verticalHigh = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = 1.0, strength01 = 1.0)
        assertTrue(verticalLow.restGateScale >= verticalMid.restGateScale)
        assertTrue(verticalMid.restGateScale >= verticalHigh.restGateScale)
    }

    @Test
    fun axisKdFloorRespectedBothAxes() {
        for (axis in listOf(0.0, 1.0)) {
            for (strength in listOf(0.0, 0.5, 1.0)) {
                val profile = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = axis, strength01 = strength)
                val kp = 16.0 * profile.tiltKpScale
                val kdMapped = 0.8 * profile.tiltKdScale
                val kdEff = PhysBearingServoMath.kdWithFloor(
                    kp = kp,
                    kdMapped = kdMapped,
                    zetaMin = profile.offAxisDampingZetaMin
                )
                val kdFloor = 2.0 * profile.offAxisDampingZetaMin * sqrt(max(kp, 1.0e-9))
                assertTrue(kdEff + 1.0e-9 >= kdFloor)
            }
        }
    }

    @Test
    fun axisProfileOutputsFiniteAndBounded() {
        for (axis in listOf(0.0, 0.3, 0.6, 1.0)) {
            for (strength in listOf(0.0, 0.25, 0.5, 0.75, 1.0)) {
                val p = PhysBearingServoMath.computeAxisHoldProfile(absAxisY = axis, strength01 = strength)
                val allFinite = listOf(
                    p.verticalBlend,
                    p.horizontalBlend,
                    p.seatKpScale,
                    p.seatKdScale,
                    p.tiltKpScale,
                    p.tiltKdScale,
                    p.restGateScale,
                    p.extraOffAxisDamping,
                    p.worldSeatAccelCapScale,
                    p.worldTiltAlphaCapScale,
                    p.worldTiltAlphaEqCapScale,
                    p.microKpFloor,
                    p.microKdBoost,
                    p.offAxisDampingZetaMin
                ).all { it.isFinite() }
                assertTrue(allFinite)
                assertTrue(p.verticalBlend in 0.0..1.0)
                assertTrue(p.horizontalBlend in 0.0..1.0)
                assertTrue(p.seatKpScale > 0.0)
                assertTrue(p.seatKdScale > 0.0)
                assertTrue(p.tiltKpScale > 0.0)
                assertTrue(p.tiltKdScale > 0.0)
                assertTrue(p.restGateScale in 0.0..1.0)
                assertTrue(p.extraOffAxisDamping >= 0.0)
                assertTrue(p.worldSeatAccelCapScale >= 1.0)
                assertTrue(p.worldTiltAlphaCapScale >= 1.0)
                assertTrue(p.worldTiltAlphaEqCapScale >= 1.0)
                assertTrue(p.microKpFloor in 0.0..1.0)
                assertTrue(p.microKdBoost >= 1.0)
                assertTrue(p.offAxisDampingZetaMin >= 1.0)
            }
        }
    }

    @Test
    fun leakyBiasConvergesAndBounded() {
        var bias = Vector3d()
        val dt = 1.0 / 20.0
        val maxMag = 0.5
        repeat(160) {
            bias = PhysBearingServoMath.stepLeakyVectorBias(
                current = bias,
                error = Vector3d(1.0, 0.0, 0.0),
                dtSec = dt,
                ki = 2.0,
                leakPerSec = 0.8,
                maxMagnitude = maxMag,
                freezeIntegrate = false
            )
        }
        assertTrue(bias.length() <= maxMag + 1.0e-9)
        assertTrue(bias.x > 0.0)
        repeat(220) {
            bias = PhysBearingServoMath.stepLeakyVectorBias(
                current = bias,
                error = Vector3d(),
                dtSec = dt,
                ki = 2.0,
                leakPerSec = 0.8,
                maxMagnitude = maxMag,
                freezeIntegrate = false
            )
        }
        assertTrue(bias.length() < 1.0e-3)
    }

    @Test
    fun leakyBiasFreezeOnSaturationPreventsWindup() {
        var bias = Vector3d(0.35, 0.0, 0.0)
        val dt = 1.0 / 20.0
        repeat(120) {
            bias = PhysBearingServoMath.stepLeakyVectorBias(
                current = bias,
                error = Vector3d(8.0, 0.0, 0.0),
                dtSec = dt,
                ki = 3.0,
                leakPerSec = 0.9,
                maxMagnitude = 0.8,
                freezeIntegrate = true
            )
        }
        assertTrue(bias.length() < 0.35)
    }

    @Test
    fun biasBoundedSlewRateLimited() {
        val dt = 1.0 / 20.0
        val maxDeltaPerSec = 0.4
        val maxDelta = maxDeltaPerSec * dt
        val next = PhysBearingServoMath.stepLeakyVectorBiasBounded(
            current = Vector3d(),
            error = Vector3d(50.0, 0.0, 0.0),
            dtSec = dt,
            ki = 8.0,
            leakPerSec = 1.0,
            maxMagnitude = 10.0,
            maxDeltaPerSec = maxDeltaPerSec,
            leakBoost = 1.0,
            freezeIntegrate = false
        )
        assertTrue(next.length() <= maxDelta + 1.0e-9)
    }

    @Test
    fun biasFlipFreezeBlocksIntegrateAndLeaksDown() {
        val dt = 1.0 / 20.0
        val current = Vector3d(0.7, 0.0, 0.0)
        val freezeStep = PhysBearingServoMath.stepLeakyVectorBiasBounded(
            current = current,
            error = Vector3d(-12.0, 0.0, 0.0),
            dtSec = dt,
            ki = 4.0,
            leakPerSec = 1.5,
            maxMagnitude = 2.0,
            maxDeltaPerSec = 10.0,
            leakBoost = 2.6,
            freezeIntegrate = true
        )
        assertTrue(freezeStep.x in 0.0..0.7)

        val integrateStep = PhysBearingServoMath.stepLeakyVectorBiasBounded(
            current = current,
            error = Vector3d(-12.0, 0.0, 0.0),
            dtSec = dt,
            ki = 4.0,
            leakPerSec = 1.5,
            maxMagnitude = 2.0,
            maxDeltaPerSec = 10.0,
            leakBoost = 1.0,
            freezeIntegrate = false
        )
        assertTrue(integrateStep.x < freezeStep.x)
    }

    @Test
    fun biasUpdateOverdampedNoFlipChatter() {
        val dt = 1.0 / 20.0
        var bias = Vector3d(0.5, 0.0, 0.0)
        repeat(20) {
            bias = PhysBearingServoMath.stepLeakyVectorBiasBounded(
                current = bias,
                error = Vector3d(-0.08, 0.0, 0.0),
                dtSec = dt,
                ki = 0.6,
                leakPerSec = 3.0,
                maxMagnitude = 1.5,
                maxDeltaPerSec = 0.6,
                leakBoost = 2.5,
                freezeIntegrate = true
            )
            assertTrue(bias.x >= -1.0e-9)
        }
        assertTrue(bias.x < 0.5)
    }

    @Test
    fun restEarlyReturnRequiresBiasNearZero() {
        var state = PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        repeat(10) {
            state = PhysBearingServoMath.allowRestEarlyReturn(
                previouslyAllowed = state.active,
                previousTicks = state.ticks,
                seatRestBand = true,
                tiltRestBand = true,
                seatMicroBand = true,
                tiltMicroBand = true,
                seatBiasMagnitude = 0.25,
                tiltBiasMagnitude = 0.25,
                seatBiasZeroBand = 0.1,
                tiltBiasZeroBand = 0.1,
                enterTicksRequired = 4
            )
        }
        assertFalse(state.active)

        repeat(4) {
            state = PhysBearingServoMath.allowRestEarlyReturn(
                previouslyAllowed = state.active,
                previousTicks = state.ticks,
                seatRestBand = true,
                tiltRestBand = true,
                seatMicroBand = true,
                tiltMicroBand = true,
                seatBiasMagnitude = 0.02,
                tiltBiasMagnitude = 0.03,
                seatBiasZeroBand = 0.1,
                tiltBiasZeroBand = 0.1,
                enterTicksRequired = 4
            )
        }
        assertTrue(state.active)

        state = PhysBearingServoMath.allowRestEarlyReturn(
            previouslyAllowed = state.active,
            previousTicks = state.ticks,
            seatRestBand = true,
            tiltRestBand = true,
            seatMicroBand = true,
            tiltMicroBand = true,
            seatBiasMagnitude = 0.25,
            tiltBiasMagnitude = 0.03,
            seatBiasZeroBand = 0.1,
            tiltBiasZeroBand = 0.1,
            enterTicksRequired = 4
        )
        assertFalse(state.active)
    }

    @Test
    fun ultraSleepRequiresDualBandAndBiasZeroDwell() {
        var state = PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        repeat(8) {
            state = PhysBearingServoMath.stepUltraSleep(
                previouslyActive = state.active,
                previousTicks = state.ticks,
                ultraStable = true,
                seatBiasMagnitude = 0.22,
                tiltBiasMagnitude = 0.18,
                seatBiasZeroBand = 0.1,
                tiltBiasZeroBand = 0.1,
                enterTicksRequired = 4,
                exitMultiplier = 1.8
            )
        }
        assertFalse(state.active)

        repeat(4) {
            state = PhysBearingServoMath.stepUltraSleep(
                previouslyActive = state.active,
                previousTicks = state.ticks,
                ultraStable = true,
                seatBiasMagnitude = 0.03,
                tiltBiasMagnitude = 0.02,
                seatBiasZeroBand = 0.1,
                tiltBiasZeroBand = 0.1,
                enterTicksRequired = 4,
                exitMultiplier = 1.8
            )
        }
        assertTrue(state.active)
    }

    @Test
    fun ultraSleepHysteresisNoToggleUnderNoise() {
        var state = PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        repeat(4) {
            state = PhysBearingServoMath.stepUltraSleep(
                previouslyActive = state.active,
                previousTicks = state.ticks,
                ultraStable = true,
                seatBiasMagnitude = 0.03,
                tiltBiasMagnitude = 0.03,
                seatBiasZeroBand = 0.1,
                tiltBiasZeroBand = 0.1,
                enterTicksRequired = 4,
                exitMultiplier = 1.8
            )
        }
        assertTrue(state.active)

        val jitter = listOf(0.11, 0.16, 0.13, 0.15, 0.17)
        for (seatBias in jitter) {
            state = PhysBearingServoMath.stepUltraSleep(
                previouslyActive = state.active,
                previousTicks = state.ticks,
                ultraStable = true,
                seatBiasMagnitude = seatBias,
                tiltBiasMagnitude = 0.12,
                seatBiasZeroBand = 0.1,
                tiltBiasZeroBand = 0.1,
                enterTicksRequired = 4,
                exitMultiplier = 1.8
            )
            assertTrue(state.active)
        }

        state = PhysBearingServoMath.stepUltraSleep(
            previouslyActive = state.active,
            previousTicks = state.ticks,
            ultraStable = true,
            seatBiasMagnitude = 0.19,
            tiltBiasMagnitude = 0.12,
            seatBiasZeroBand = 0.1,
            tiltBiasZeroBand = 0.1,
            enterTicksRequired = 4,
            exitMultiplier = 1.8
        )
        assertFalse(state.active)
    }

    @Test
    fun holdAcquireMappingMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.holdAcquireTicks <= mid.holdAcquireTicks && mid.holdAcquireTicks <= high.holdAcquireTicks)
        assertTrue(low.holdSpawnSettleTicks <= mid.holdSpawnSettleTicks && mid.holdSpawnSettleTicks <= high.holdSpawnSettleTicks)
        assertTrue(low.holdAcquireKpScale >= mid.holdAcquireKpScale && mid.holdAcquireKpScale >= high.holdAcquireKpScale)
        assertTrue(low.holdAcquireKdBoost <= mid.holdAcquireKdBoost && mid.holdAcquireKdBoost <= high.holdAcquireKdBoost)
        assertTrue(low.holdAcquireTicks in 5..12)
        assertTrue(mid.holdAcquireTicks in 5..12)
        assertTrue(high.holdAcquireTicks in 5..12)
        assertTrue(low.holdSpawnSettleTicks in 8..16)
        assertTrue(mid.holdSpawnSettleTicks in 8..16)
        assertTrue(high.holdSpawnSettleTicks in 8..16)
        assertTrue(low.holdAcquireKpScale in 0.40..0.62)
        assertTrue(mid.holdAcquireKpScale in 0.40..0.62)
        assertTrue(high.holdAcquireKpScale in 0.40..0.62)
        assertTrue(low.holdAcquireKdBoost in 1.30..2.00)
        assertTrue(mid.holdAcquireKdBoost in 1.30..2.00)
        assertTrue(high.holdAcquireKdBoost in 1.30..2.00)
    }

    @Test
    fun settleMappingReducedAndMonotonic() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.holdPostBrakeSettleTicks in 3..8)
        assertTrue(mid.holdPostBrakeSettleTicks in 3..8)
        assertTrue(high.holdPostBrakeSettleTicks in 3..8)
        assertTrue(low.holdPostBrakeSettleTicks <= mid.holdPostBrakeSettleTicks)
        assertTrue(mid.holdPostBrakeSettleTicks <= high.holdPostBrakeSettleTicks)
        assertTrue(low.holdPostBrakeKdBoost in 1.10..1.55)
        assertTrue(mid.holdPostBrakeKdBoost in 1.10..1.55)
        assertTrue(high.holdPostBrakeKdBoost in 1.10..1.55)
        assertTrue(low.holdPostBrakeKdBoost <= mid.holdPostBrakeKdBoost)
        assertTrue(mid.holdPostBrakeKdBoost <= high.holdPostBrakeKdBoost)
    }

    @Test
    fun holdSettleDampingMonotonicWithStrength() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)
        assertTrue(low.holdPostBrakeSettleTicks <= mid.holdPostBrakeSettleTicks)
        assertTrue(mid.holdPostBrakeSettleTicks <= high.holdPostBrakeSettleTicks)
        assertTrue(low.holdPostBrakeKdBoost <= mid.holdPostBrakeKdBoost)
        assertTrue(mid.holdPostBrakeKdBoost <= high.holdPostBrakeKdBoost)
    }

    @Test
    fun strengthMonotonicBiasAuthority() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.holdSeatBiasKi <= mid.holdSeatBiasKi && mid.holdSeatBiasKi <= high.holdSeatBiasKi)
        assertTrue(low.holdSeatBiasMaxAccel <= mid.holdSeatBiasMaxAccel && mid.holdSeatBiasMaxAccel <= high.holdSeatBiasMaxAccel)
        assertTrue(low.holdTiltBiasKi <= mid.holdTiltBiasKi && mid.holdTiltBiasKi <= high.holdTiltBiasKi)
        assertTrue(low.holdTiltBiasMaxAlpha <= mid.holdTiltBiasMaxAlpha && mid.holdTiltBiasMaxAlpha <= high.holdTiltBiasMaxAlpha)
        assertTrue(low.holdSeatBiasLeak <= mid.holdSeatBiasLeak && mid.holdSeatBiasLeak <= high.holdSeatBiasLeak)
        assertTrue(low.holdTiltBiasLeak <= mid.holdTiltBiasLeak && mid.holdTiltBiasLeak <= high.holdTiltBiasLeak)
        assertTrue(low.holdBiasFreezeTicks <= mid.holdBiasFreezeTicks && mid.holdBiasFreezeTicks <= high.holdBiasFreezeTicks)
        assertTrue(
            low.holdBiasLeakBoostOnFlip <= mid.holdBiasLeakBoostOnFlip &&
                mid.holdBiasLeakBoostOnFlip <= high.holdBiasLeakBoostOnFlip
        )
        assertTrue(
            low.holdSeatBiasSlewPerSec <= mid.holdSeatBiasSlewPerSec &&
                mid.holdSeatBiasSlewPerSec <= high.holdSeatBiasSlewPerSec
        )
        assertTrue(
            low.holdTiltBiasSlewPerSec <= mid.holdTiltBiasSlewPerSec &&
                mid.holdTiltBiasSlewPerSec <= high.holdTiltBiasSlewPerSec
        )
        assertTrue(low.holdRestBiasZeroBandScale >= mid.holdRestBiasZeroBandScale)
        assertTrue(mid.holdRestBiasZeroBandScale >= high.holdRestBiasZeroBandScale)
    }

    @Test
    fun allNewParamsFiniteBounded() {
        for (strength in listOf(0.0, 0.25, 0.5, 0.75, 1.0)) {
            val p = PhysBearingServoMath.mapFollowStrength(strength01 = strength, sliderScale = 1.0)
            assertTrue(p.holdSeatBiasKi.isFinite() && p.holdSeatBiasKi >= 0.0)
            assertTrue(p.holdSeatBiasLeak.isFinite() && p.holdSeatBiasLeak > 0.0)
            assertTrue(p.holdSeatBiasMaxAccel.isFinite() && p.holdSeatBiasMaxAccel >= 0.0)
            assertTrue(p.holdTiltBiasKi.isFinite() && p.holdTiltBiasKi >= 0.0)
            assertTrue(p.holdTiltBiasLeak.isFinite() && p.holdTiltBiasLeak > 0.0)
            assertTrue(p.holdTiltBiasMaxAlpha.isFinite() && p.holdTiltBiasMaxAlpha >= 0.0)
            assertTrue(p.holdBiasFreezeTicks >= 0)
            assertTrue(p.holdBiasLeakBoostOnFlip.isFinite() && p.holdBiasLeakBoostOnFlip >= 1.0)
            assertTrue(p.holdSeatBiasSlewPerSec.isFinite() && p.holdSeatBiasSlewPerSec >= 0.0)
            assertTrue(p.holdTiltBiasSlewPerSec.isFinite() && p.holdTiltBiasSlewPerSec >= 0.0)
            assertTrue(p.holdAcquireTicks >= 0)
            assertTrue(p.holdSpawnSettleTicks >= 0)
            assertTrue(p.holdAcquireKpScale.isFinite() && p.holdAcquireKpScale in 0.0..1.0)
            assertTrue(p.holdAcquireKdBoost.isFinite() && p.holdAcquireKdBoost >= 1.0)
            assertTrue(p.holdPostBrakeSettleTicks in 3..8)
            assertTrue(p.holdPostBrakeKdBoost.isFinite() && p.holdPostBrakeKdBoost >= 1.0)
            assertTrue(p.holdRestBiasZeroBandScale.isFinite() && p.holdRestBiasZeroBandScale in 0.0..1.0)
            assertTrue(p.fixedHoldSeatAuthorityFloor.isFinite() && p.fixedHoldSeatAuthorityFloor in 0.0..1.0)
            assertTrue(p.fixedHoldTiltAuthorityFloor.isFinite() && p.fixedHoldTiltAuthorityFloor in 0.0..1.0)
            assertTrue(p.fixedHoldSeatKpFloorScale.isFinite() && p.fixedHoldSeatKpFloorScale in 0.0..1.0)
            assertTrue(p.fixedHoldTiltStiffnessFloor.isFinite() && p.fixedHoldTiltStiffnessFloor in 0.0..1.0)
            assertTrue(p.fixedOffAxisDampingZetaMin.isFinite() && p.fixedOffAxisDampingZetaMin >= 1.0)
        }
    }

    @Test
    fun heavyLeverBlendMonotonic() {
        val start = 1.5
        val end = 8.0
        val b0 = PhysBearingServoMath.heavyLeverBlend(0.0, start, end)
        val b1 = PhysBearingServoMath.heavyLeverBlend(1.5, start, end)
        val b2 = PhysBearingServoMath.heavyLeverBlend(3.0, start, end)
        val b3 = PhysBearingServoMath.heavyLeverBlend(8.0, start, end)
        val b4 = PhysBearingServoMath.heavyLeverBlend(20.0, start, end)
        assertEquals(0.0, b0, 1.0e-12)
        assertEquals(0.0, b1, 1.0e-12)
        assertTrue(b0 <= b1 && b1 <= b2 && b2 <= b3 && b3 <= b4)
        assertEquals(1.0, b3, 1.0e-12)
        assertEquals(1.0, b4, 1.0e-12)
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
