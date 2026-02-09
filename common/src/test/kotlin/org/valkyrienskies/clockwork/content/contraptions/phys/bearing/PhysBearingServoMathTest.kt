package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.math.sign

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
        assertTrue(
            low.trackPosAssistOmegaLimit < mid.trackPosAssistOmegaLimit &&
                mid.trackPosAssistOmegaLimit < high.trackPosAssistOmegaLimit
        )
    }

    @Test
    fun strengthMonotonicHoldAuthority() {
        val low = PhysBearingServoMath.mapFollowStrength(strength01 = 0.0, sliderScale = 1.0)
        val mid = PhysBearingServoMath.mapFollowStrength(strength01 = 0.5, sliderScale = 1.0)
        val high = PhysBearingServoMath.mapFollowStrength(strength01 = 1.0, sliderScale = 1.0)

        assertTrue(low.holdPosGain < mid.holdPosGain && mid.holdPosGain < high.holdPosGain)
        assertTrue(low.holdOmegaGain < mid.holdOmegaGain && mid.holdOmegaGain < high.holdOmegaGain)
        assertTrue(low.holdPosOmegaLimit < mid.holdPosOmegaLimit && mid.holdPosOmegaLimit < high.holdPosOmegaLimit)
        assertTrue(low.holdMaxAlpha < mid.holdMaxAlpha && mid.holdMaxAlpha < high.holdMaxAlpha)
    }

    @Test
    fun holdPhaseRestoringAlpha() {
        val p = PhysBearingServoMath.mapFollowStrength(strength01 = 0.75, sliderScale = 1.0)
        val deadbandError = 0.002
        val deadbandOmega = 0.02

        val alphaPositiveError = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = 0.10,
            omegaActualRadSec = 0.0,
            holdPosGain = p.holdPosGain,
            holdOmegaGain = p.holdOmegaGain,
            holdPosOmegaLimit = p.holdPosOmegaLimit,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaNegativeError = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = -0.10,
            omegaActualRadSec = 0.0,
            holdPosGain = p.holdPosGain,
            holdOmegaGain = p.holdOmegaGain,
            holdPosOmegaLimit = p.holdPosOmegaLimit,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaPositiveOmega = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = 0.0,
            omegaActualRadSec = 0.20,
            holdPosGain = p.holdPosGain,
            holdOmegaGain = p.holdOmegaGain,
            holdPosOmegaLimit = p.holdPosOmegaLimit,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaNegativeOmega = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = 0.0,
            omegaActualRadSec = -0.20,
            holdPosGain = p.holdPosGain,
            holdOmegaGain = p.holdOmegaGain,
            holdPosOmegaLimit = p.holdPosOmegaLimit,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )

        assertTrue(alphaPositiveError > 0.0)
        assertTrue(alphaNegativeError < 0.0)
        assertTrue(alphaPositiveOmega < 0.0)
        assertTrue(alphaNegativeOmega > 0.0)
    }

    @Test
    fun holdDeadbandNoChatter() {
        val p = PhysBearingServoMath.mapFollowStrength(strength01 = 0.6, sliderScale = 1.0)
        val deadbandError = 0.002
        val deadbandOmega = 0.02
        val alphaDeadband = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = deadbandError * 0.5,
            omegaActualRadSec = deadbandOmega * 0.5,
            holdPosGain = p.holdPosGain,
            holdOmegaGain = p.holdOmegaGain,
            holdPosOmegaLimit = p.holdPosOmegaLimit,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )
        val alphaOutsideBand = PhysBearingServoMath.computeHoldAlpha(
            holdErrorRad = deadbandError * 1.5,
            omegaActualRadSec = deadbandOmega * 1.5,
            holdPosGain = p.holdPosGain,
            holdOmegaGain = p.holdOmegaGain,
            holdPosOmegaLimit = p.holdPosOmegaLimit,
            holdErrorDeadbandRad = deadbandError,
            holdOmegaDeadbandRadSec = deadbandOmega
        )

        assertTrue(alphaDeadband == 0.0)
        assertTrue(alphaOutsideBand.isFinite())
        assertTrue(abs(alphaOutsideBand) < p.holdOmegaGain * p.holdPosOmegaLimit + 10.0)
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
