package org.valkyrienskies.clockwork.content.forces.contraption

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class BearingControllerStabilityTest {

    @Test
    fun alignmentFactorFadesMonotonicallyBetweenThresholds() {
        val start = 5.0
        val stop = 35.0

        val factors = listOf(0.0, 5.0, 10.0, 20.0, 30.0, 35.0, 40.0).map {
            BearingController.computeUnlockedAlignmentFactor(it, start, stop)
        }

        assertEquals(1.0, factors.first(), 1e-12)
        assertEquals(0.0, factors.last(), 1e-12)
        assertTrue(factors.zipWithNext().all { (left, right) -> left >= right })
        assertTrue(factors[3] in 0.0..1.0)
    }

    @Test
    fun torqueScalarUsesAngularAccelerationClamp() {
        val torque = BearingController.computeUnlockedTorqueScalar(
            effectiveInertia = 100.0,
            omegaErrorAxis = 10.0,
            omegaErrorMultiplier = 5.0,
            maxAngularAcceleration = 20.0,
            alignmentFactor = 0.5
        )
        // alpha = clamp(10 * 5, +/-20) = 20, torque = 100 * 20 * 0.5 = 1000
        assertEquals(1000.0, torque, 1e-9)
    }

    @Test
    fun nonFiniteInputsReturnZeroOutputs() {
        val omegaErr = BearingController.computeUnlockedOmegaErrorAxis(
            desiredOmegaAxis = Double.NaN,
            actualOmegaAxis = 1.0,
            resistanceMultiplier = 1.0
        )
        assertEquals(0.0, omegaErr, 0.0)

        val torque = BearingController.computeUnlockedTorqueScalar(
            effectiveInertia = Double.POSITIVE_INFINITY,
            omegaErrorAxis = 1.0,
            omegaErrorMultiplier = 1.0,
            maxAngularAcceleration = 10.0,
            alignmentFactor = 1.0
        )
        assertEquals(0.0, torque, 0.0)
    }

    @Test
    fun runtimeFloorStrengthensWeakConfigValues() {
        val boosted = BearingController.applyRuntimeFloor(20.0, 60.0)
        val unchanged = BearingController.applyRuntimeFloor(90.0, 60.0)

        assertEquals(60.0, boosted, 1e-9)
        assertEquals(90.0, unchanged, 1e-9)
    }
}
