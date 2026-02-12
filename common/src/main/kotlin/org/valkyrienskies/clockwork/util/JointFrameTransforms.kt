package org.valkyrienskies.clockwork.util

import net.minecraft.server.level.ServerLevel
import org.joml.Matrix4dc
import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.core.api.ships.Ship
import org.valkyrienskies.mod.common.shipObjectWorld

fun pointToWorld(level: ServerLevel, shipId: Long?, pointInEndpointFrame: Vector3dc): Vector3d {
    val fromShipToWorld = resolveShip(level, shipId)?.shipToWorld
    return convertPointWithTransforms(pointInEndpointFrame, fromShipToWorld, null)
}

fun pointFromWorld(level: ServerLevel, shipId: Long?, pointInWorld: Vector3dc): Vector3d {
    val toWorldToShip = resolveShip(level, shipId)?.worldToShip
    return convertPointWithTransforms(pointInWorld, null, toWorldToShip)
}

fun convertPoint(level: ServerLevel, fromShipId: Long?, toShipId: Long?, point: Vector3dc): Vector3d {
    val fromShipToWorld = resolveShip(level, fromShipId)?.shipToWorld
    val toWorldToShip = resolveShip(level, toShipId)?.worldToShip
    return convertPointWithTransforms(point, fromShipToWorld, toWorldToShip)
}

fun rotToWorld(level: ServerLevel, shipId: Long?, rotInEndpointFrame: Quaterniondc): Quaterniond {
    val fromShipToWorldRot = resolveShip(level, shipId)?.shipToWorld?.getNormalizedRotation(Quaterniond())
    return convertRotWithTransforms(rotInEndpointFrame, fromShipToWorldRot, null)
}

fun rotFromWorld(level: ServerLevel, shipId: Long?, rotInWorld: Quaterniondc): Quaterniond {
    val toWorldToShipRot = resolveShip(level, shipId)?.worldToShip?.getNormalizedRotation(Quaterniond())
    return convertRotWithTransforms(rotInWorld, null, toWorldToShipRot)
}

fun convertRot(level: ServerLevel, fromShipId: Long?, toShipId: Long?, rot: Quaterniondc): Quaterniond {
    val fromShipToWorldRot = resolveShip(level, fromShipId)?.shipToWorld?.getNormalizedRotation(Quaterniond())
    val toWorldToShipRot = resolveShip(level, toShipId)?.worldToShip?.getNormalizedRotation(Quaterniond())
    return convertRotWithTransforms(rot, fromShipToWorldRot, toWorldToShipRot)
}

internal fun convertPointWithTransforms(
    point: Vector3dc,
    fromShipToWorld: Matrix4dc?,
    toWorldToShip: Matrix4dc?
): Vector3d {
    val inWorld = if (fromShipToWorld != null) {
        fromShipToWorld.transformPosition(point, Vector3d())
    } else {
        Vector3d(point)
    }
    return if (toWorldToShip != null) {
        toWorldToShip.transformPosition(inWorld, Vector3d())
    } else {
        inWorld
    }
}

internal fun convertRotWithTransforms(
    rot: Quaterniondc,
    fromShipToWorldRot: Quaterniondc?,
    toWorldToShipRot: Quaterniondc?
): Quaterniond {
    val inWorld = if (fromShipToWorldRot != null) {
        Quaterniond(fromShipToWorldRot).mul(rot).normalize()
    } else {
        Quaterniond(rot).normalize()
    }
    return if (toWorldToShipRot != null) {
        Quaterniond(toWorldToShipRot).mul(inWorld).normalize()
    } else {
        inWorld
    }
}

private fun resolveShip(level: ServerLevel, shipId: Long?): Ship? {
    if (shipId == null || shipId < 0L) {
        return null
    }
    return (level.shipObjectWorld.loadedShips.getById(shipId) as? Ship)
        ?: (level.shipObjectWorld.allShips.getById(shipId) as? Ship)
}
