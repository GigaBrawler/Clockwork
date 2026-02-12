package org.valkyrienskies.clockwork.util

import org.joml.Matrix4d
import org.joml.Quaterniond
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class JointFrameTransformsTest {

    @Test
    fun pointRoundtripAcrossTwoFramesIsStable() {
        val aToWorld = Matrix4d()
            .translate(10.0, -3.0, 4.0)
            .rotateXYZ(0.1, 0.4, -0.2)
        val bToWorld = Matrix4d()
            .translate(-2.0, 8.0, 1.0)
            .rotateXYZ(-0.3, 0.2, 0.6)

        val worldToA = Matrix4d(aToWorld).invert()
        val worldToB = Matrix4d(bToWorld).invert()

        val pointInA = Vector3d(1.25, -0.5, 4.75)
        val pointInB = convertPointWithTransforms(pointInA, aToWorld, worldToB)
        val roundtrip = convertPointWithTransforms(pointInB, bToWorld, worldToA)

        assertEquals(pointInA.x, roundtrip.x, 1e-9)
        assertEquals(pointInA.y, roundtrip.y, 1e-9)
        assertEquals(pointInA.z, roundtrip.z, 1e-9)
    }

    @Test
    fun rotationRoundtripAcrossTwoFramesIsStable() {
        val aToWorldRot = Quaterniond().rotateXYZ(0.2, -0.5, 0.1)
        val bToWorldRot = Quaterniond().rotateXYZ(-0.1, 0.3, 0.6)
        val worldToARot = Quaterniond(aToWorldRot).conjugate()
        val worldToBRot = Quaterniond(bToWorldRot).conjugate()

        val rotInA = Quaterniond().rotateXYZ(0.4, 0.1, -0.2).normalize()
        val rotInB = convertRotWithTransforms(rotInA, aToWorldRot, worldToBRot)
        val roundtrip = convertRotWithTransforms(rotInB, bToWorldRot, worldToARot)

        assertEquals(1.0, kotlin.math.abs(Quaterniond(rotInA).dot(roundtrip)), 1e-9)
    }

    @Test
    fun nullTransformsBehaveAsWorldFrame() {
        val point = Vector3d(3.0, 2.0, -7.0)
        val rot = Quaterniond().rotateXYZ(0.1, 0.2, 0.3).normalize()

        val pointResult = convertPointWithTransforms(point, null, null)
        val rotResult = convertRotWithTransforms(rot, null, null)

        assertEquals(point.x, pointResult.x, 0.0)
        assertEquals(point.y, pointResult.y, 0.0)
        assertEquals(point.z, pointResult.z, 0.0)
        assertEquals(1.0, kotlin.math.abs(rot.dot(rotResult)), 1e-9)
    }
}
