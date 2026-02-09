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
}
