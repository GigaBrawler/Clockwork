package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

internal object PhysBearingAngleMath {
    private const val TWO_PI = Math.PI * 2.0

    internal fun unwrapNearReference(angleRad: Double, referenceRad: Double): Double {
        if (!angleRad.isFinite() || !referenceRad.isFinite()) {
            return referenceRad
        }
        val shortestDelta = Math.IEEEremainder(angleRad - referenceRad, TWO_PI)
        return referenceRad + shortestDelta
    }

    internal fun stepToward(currentRad: Double, targetRad: Double, maxStepRad: Double): Double {
        if (!currentRad.isFinite() || !targetRad.isFinite()) {
            return currentRad
        }
        if (!maxStepRad.isFinite() || maxStepRad <= 0.0) {
            return targetRad
        }
        val delta = targetRad - currentRad
        if (abs(delta) <= maxStepRad) {
            return targetRad
        }
        return currentRad + (sign(delta) * min(abs(delta), maxStepRad))
    }

    internal fun shouldPushUpdate(
        lastPushedAngleRad: Double,
        nextAngleRad: Double,
        epsilonRad: Double,
        force: Boolean
    ): Boolean {
        if (force) {
            return true
        }
        if (!nextAngleRad.isFinite()) {
            return false
        }
        if (!lastPushedAngleRad.isFinite()) {
            return true
        }
        return abs(nextAngleRad - lastPushedAngleRad) > epsilonRad
    }

    internal fun initializeAppliedAngle(desiredAngleRad: Double, actualAngleRad: Double?): Double {
        if (!desiredAngleRad.isFinite()) {
            return 0.0
        }
        val actual = actualAngleRad
        if (actual == null || !actual.isFinite()) {
            return desiredAngleRad
        }
        return unwrapNearReference(actual, desiredAngleRad)
    }
}
