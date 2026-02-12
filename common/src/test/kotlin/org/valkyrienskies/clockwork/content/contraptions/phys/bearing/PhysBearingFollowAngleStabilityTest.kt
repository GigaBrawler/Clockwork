package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.abs

class PhysBearingFollowAngleStabilityTest {

    @Test
    fun unwrapNearReferenceKeepsBoundaryTransitionContinuous() {
        val reference = Math.toRadians(359.0)
        val wrapped = Math.toRadians(1.0)
        val unwrapped = PhysBearingAngleMath.unwrapNearReference(wrapped, reference)

        assertEquals(Math.toRadians(361.0), unwrapped, 1e-9)
        assertTrue(abs(unwrapped - reference) < Math.PI)
    }

    @Test
    fun stepTowardUsesBoundedProgress() {
        val next = PhysBearingAngleMath.stepToward(
            currentRad = 0.0,
            targetRad = 3.0,
            maxStepRad = 0.25
        )
        assertEquals(0.25, next, 1e-9)
    }

    @Test
    fun stepTowardConvergesWithoutOvershoot() {
        var current = 0.0
        val target = 1.0
        repeat(10) {
            current = PhysBearingAngleMath.stepToward(current, target, 0.2)
        }

        assertEquals(target, current, 1e-9)
    }

    @Test
    fun pushUpdateUsesEpsilonDedup() {
        val epsilon = 1e-4
        assertFalse(PhysBearingAngleMath.shouldPushUpdate(1.0, 1.0 + 1e-6, epsilon, force = false))
        assertTrue(PhysBearingAngleMath.shouldPushUpdate(1.0, 1.0 + 1e-3, epsilon, force = false))
        assertTrue(PhysBearingAngleMath.shouldPushUpdate(1.0, 1.0 + 1e-6, epsilon, force = true))
    }

    @Test
    fun initializeAppliedAngleUnwrapsActualAroundDesired() {
        val desired = Math.toRadians(5.0)
        val actual = Math.toRadians(350.0)
        val initialized = PhysBearingAngleMath.initializeAppliedAngle(desired, actual)

        assertTrue(abs(initialized - desired) < Math.toRadians(30.0))
        assertEquals(desired, PhysBearingAngleMath.initializeAppliedAngle(desired, null), 1e-9)
    }
}
