package org.valkyrienskies.clockwork.content.contraptions.phys.slicker

import com.simibubi.create.api.behaviour.movement.MovementBehaviour
import com.simibubi.create.content.contraptions.ControlledContraptionEntity
import com.simibubi.create.content.contraptions.StructureTransform
import com.simibubi.create.content.contraptions.TranslatingContraption
import com.simibubi.create.content.contraptions.bearing.BearingContraption
import com.simibubi.create.content.contraptions.bearing.StabilizedContraption
import com.simibubi.create.content.contraptions.behaviour.MovementContext
import com.simibubi.create.content.contraptions.gantry.GantryContraption
import com.simibubi.create.content.contraptions.piston.LinearActuatorBlockEntity
import com.simibubi.create.content.contraptions.piston.PistonContraption
import com.simibubi.create.content.contraptions.pulley.PulleyContraption
import net.createmod.catnip.math.VecHelper
import net.minecraft.core.BlockPos
import net.minecraft.core.Direction
import net.minecraft.nbt.CompoundTag
import net.minecraft.server.level.ServerLevel
import net.minecraft.world.level.block.DirectionalBlock
import net.minecraft.world.level.block.SupportType
import net.minecraft.world.level.block.state.BlockState
import net.minecraft.world.level.block.state.properties.BlockStateProperties
import net.minecraft.world.phys.AABB
import net.minecraft.world.phys.Vec3
import org.joml.Quaterniond
import org.joml.Quaterniondc
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.clockwork.ClockworkMod
import org.valkyrienskies.clockwork.mixin.accessors.IMixinPistonContraption
import org.valkyrienskies.clockwork.mixinduck.MixinAbstractContraptionEntityDuck
import org.valkyrienskies.clockwork.util.addJointPersistent
import org.valkyrienskies.clockwork.util.buildPersistentKeyOwnerRef
import org.valkyrienskies.clockwork.util.ClockworkConstants
import org.valkyrienskies.clockwork.util.getPersistentKeyForRuntimeId
import org.valkyrienskies.clockwork.util.getRuntimeIdForPersistentKey
import org.valkyrienskies.clockwork.util.gtpa
import org.valkyrienskies.clockwork.util.newPersistentJointKey
import org.valkyrienskies.clockwork.util.removeJointPersistent
import org.valkyrienskies.clockwork.util.resolveRuntimeJointId
import org.valkyrienskies.clockwork.util.updateJointPersistent
import org.valkyrienskies.clockwork.util.convertPoint
import org.valkyrienskies.core.api.ships.ServerShip
import org.valkyrienskies.core.api.ships.Ship
import org.valkyrienskies.core.internal.joints.VSFixedJoint
import org.valkyrienskies.core.internal.joints.VSJointMaxForceTorque
import org.valkyrienskies.core.internal.joints.VSJointPose
import org.valkyrienskies.core.impl.util.serialization.VSJacksonUtil
import org.valkyrienskies.mod.common.*
import org.valkyrienskies.mod.common.util.toJOML
import org.valkyrienskies.mod.common.util.toJOMLD
import org.valkyrienskies.mod.common.util.toMinecraft
import org.valkyrienskies.mod.mixinducks.mod_compat.create.IMixinControlledContraptionEntity
import kotlin.math.abs


class SlickerMovementBehavior : MovementBehaviour {

    var isStopped = true

    override fun tick(context: MovementContext) {
        if (context.world == null || context.world.isClientSide) return

        var data: CompoundTag = context.blockEntityData.getCompound(ClockworkConstants.Nbt.CONDENSED_DATA)

        if (!data.isEmpty && data.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)) {
            if (!isStopped) {
                doUpdateConstraint(context, null, null)
            }
        } else {
            if (context.state.getValue(SlickerBlock.EXTENDED)) {
                context.blockEntityData.put(ClockworkConstants.Nbt.CONDENSED_DATA, CompoundTag())
                isAttachedToShipOrWorld(true, context.world as ServerLevel, context.localPos.toJOMLD(), context.rotation.apply(Vec3.atLowerCornerOf(context.state.getValue(
                    DirectionalBlock.FACING).normal)).toJOML(), context.blockEntityData.getCompound(ClockworkConstants.Nbt.CONDENSED_DATA))
            }
        }
    }

    fun getFacingAxis(context: MovementContext): Direction.Axis? {
        var axis: Direction.Axis? = null
        if (context.contraption is PistonContraption) axis =
            (context.contraption as IMixinPistonContraption).orientation.axis
        if (context.contraption is PulleyContraption) axis = Direction.Axis.Y
        if (context.contraption is GantryContraption) axis = (context.contraption as GantryContraption).facing.axis

        return axis
    }

    override fun onSpeedChanged(context: MovementContext, oldMotion: Vec3, motion: Vec3) {
        val axis: Direction.Axis? = getFacingAxis(context)
        isStopped = if (axis != null) {
            val axisMotion = Math.abs(VecHelper.getCoordinate(motion, axis))
            axisMotion < 0.001
        } else {
            motion == Vec3.ZERO
        }
        val a = Vector3d(1.0, 45.0, 1.0)
    }

    private fun getAssembleNextTick(context: MovementContext): Boolean {
        var result = false
        if (context.contraption.entity is ControlledContraptionEntity) {
            if (context.contraption is TranslatingContraption) {
                result =
                    ((context.contraption.entity as IMixinControlledContraptionEntity).grabController() as LinearActuatorBlockEntity).assembleNextTick
            }
            if (context.contraption is BearingContraption || context.contraption is StabilizedContraption) {
//                result =
//                    ((context.contraption.entity as IMixinControlledContraptionEntity).grabController() as IMixinMechanicalBearingTileEntity).isAssembleNextTick
            }
        }
        return result
    }

    override fun startMoving(context: MovementContext?) {
        isStopped = false
    }

    override fun stopMoving(context: MovementContext) {
        isStopped = true
        var position: Vector3d? = null
        var quaterniond: Quaterniond? = null
        val extraData: CompoundTag = context.blockEntityData.getCompound(ClockworkConstants.Nbt.CONDENSED_DATA)
        var distance = DISTANCE_BUFFER
        if (extraData.contains(ClockworkConstants.Nbt.SHIP_SLICKER_DISTANCE)) distance = extraData.getDouble(ClockworkConstants.Nbt.SHIP_SLICKER_DISTANCE)
        val myDir = context.state.getValue(DirectionalBlock.FACING)
        val myDirNormal: Vec3 = (Vec3.atLowerCornerOf(myDir.normal).toJOML().mul(.5)).toMinecraft()
        if (!getAssembleNextTick(context)) {
            if (context.state.getValue(BlockStateProperties.POWERED)) extraData.putBoolean(
                ClockworkConstants.Nbt.SHIP_STICKER_ALREADY_POWERED,
                true
            )
            val structureTransform: StructureTransform =
                (context.contraption.entity as MixinAbstractContraptionEntityDuck).structureTransform
            position =
                Vec3.atCenterOf(structureTransform.apply(context.localPos))
                    .add(structureTransform.applyWithoutOffsetUncentered(myDirNormal)).toJOML()

            if (distance < DISTANCE_BUFFER) {
                position.add(
                        structureTransform.applyWithoutOffsetUncentered(
                                Vec3.atLowerCornerOf(
                                    myDir.normal
                            ).scale(distance / -1 + DISTANCE_BUFFER)
                        ).toJOML()
                )
            }
            if (context.contraption is BearingContraption) {
                val tempQuat: Quaterniond = Vec3.atLowerCornerOf(structureTransform.applyWithoutOffset(context.localPos)).toJOML().rotationTo(
                            Vec3.atLowerCornerOf(context.localPos).toJOML(), Quaterniond())
                quaterniond = Quaterniond()
                tempQuat.mul(mapper.readValue(extraData.getByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT), VSFixedJoint::class.java).pose0.rot, quaterniond)
            }
        }
        if (!extraData.isEmpty) {
            if (extraData.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID)) {
                doUpdateConstraint(context, position, quaterniond)
            }
        }
    }


    private fun doUpdateConstraint(context: MovementContext, ship1Pos: Vector3dc?, ship1Rot: Quaterniond?) {
        if (context.world.isClientSide) return

        var ship1: Ship? = null
        var ship2: Ship? = null
        var distance = DISTANCE_BUFFER

        var realShip1Pos: Vector3d? = ship1Pos as Vector3d?
        var realShip1Rot: Quaterniond? = ship1Rot

        val extraData: CompoundTag = context.blockEntityData.getCompound(ClockworkConstants.Nbt.CONDENSED_DATA)
        if (!extraData.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID) ||
            !extraData.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)
        ) {
            return
        }

        val serverLevel = context.world as ServerLevel
        val persistentKey = getAttachmentPersistentKey(extraData)
        var runtimeJointId =
            serverLevel.gtpa.resolveRuntimeJointId(extraData.getInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID))
        if (serverLevel.gtpa.getJointById(runtimeJointId) == null) {
            val reboundJointId = persistentKey
                ?.let(serverLevel.gtpa::getRuntimeIdForPersistentKey)
                ?.let(serverLevel.gtpa::resolveRuntimeJointId)
            if (reboundJointId == null || serverLevel.gtpa.getJointById(reboundJointId) == null) {
                return
            }
            runtimeJointId = reboundJointId
        }
        extraData.putInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID, runtimeJointId)
        if (persistentKey == null) {
            serverLevel.gtpa.getPersistentKeyForRuntimeId(runtimeJointId)?.let {
                extraData.putString(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY, it)
            }
        }

        var ship2Pos: Vector3d? = null
        var ship2Rot: Quaterniond? = null

        val attachConstraintData = extraData.getByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)
        val previousConstraint = mapper.readValue(attachConstraintData, VSFixedJoint::class.java)

        distance = 1.0
        ship1 = previousConstraint.shipId0?.let { context.world.shipObjectWorld.loadedShips.getById(it) }
        ship2 = previousConstraint.shipId1?.let { context.world.shipObjectWorld.loadedShips.getById(it) }
        ship2Pos = Vector3d(previousConstraint.pose1.pos)
        ship2Rot = Quaterniond(previousConstraint.pose1.rot)

        if (ship1 == null && ship2 == null) return

        if (realShip1Pos == null) {
            realShip1Pos = Vector3d(previousConstraint.pose0.pos)
            if (context.contraption is StabilizedContraption) {
                realShip1Pos.add(
                    Vec3.atLowerCornerOf((context.contraption as StabilizedContraption).facing.normal).toJOML()
                        .mul(0.125)
                )
            }
            realShip1Pos.add(context.motion.toJOML())
        }
        if (realShip1Rot == null) {
            realShip1Rot = Quaterniond(previousConstraint.pose0.rot)
            val rotationState = context.contraption.entity.rotationState
            if (rotationState != null) {
                realShip1Rot = Quaterniond().setFromNormalized(rotationState.asMatrix().asMatrix4f).mul(realShip1Rot)
            }
        }

        val updatedConstraint = makeConstraint(
            realShip1Pos,
            Vector3d(realShip1Pos),
            ship1,
            ship2,
            serverLevel,
            realShip1Rot,
            ship2Rot,
            ship2Pos
        ) ?: return

        if (jointsEquivalent(previousConstraint, updatedConstraint)) {
            return
        }
        serverLevel.gtpa.updateJointPersistent(runtimeJointId, updatedConstraint)
        extraData.putByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT, mapper.writeValueAsBytes(updatedConstraint))
        extraData.putDouble(ClockworkConstants.Nbt.SHIP_SLICKER_DISTANCE, distance)

        ClockworkMod.LOGGER.info(
            "Updated constraint from ship ${updatedConstraint.shipId0} to ${updatedConstraint.shipId1} using points ${updatedConstraint.pose0.pos} && ${updatedConstraint.pose1.pos}"
        )
    }

    companion object {
        val mapper = VSJacksonUtil.defaultMapper
        const val DISTANCE_BUFFER = 1.05
        private const val JOINT_UPDATE_EPSILON = 1e-5

        fun isAttachedToShipOrWorld(
            attach: Boolean,
            level: ServerLevel?,
            myPosCentered: Vector3dc,
            myDirNormal: Vector3dc,
            compoundTag: CompoundTag
        ): Boolean {
            var result = false
            if (level == null) return false
            val ship: ServerShip? = level.getShipManagingPos(myPosCentered)
            var ship2: Ship? = null
            val tempDirNormal: Vector3d = myDirNormal.mul(.75, Vector3d())
            val searchPos: Vector3d = Vector3d(myPosCentered).add(tempDirNormal)
            ship?.shipToWorld?.transformPosition(searchPos, searchPos)
            var searchBlockPos: BlockPos = BlockPos.containing(searchPos.toMinecraft())
            val worldBlockState: BlockState = level.getBlockState(searchBlockPos)
            var distance = 0.0
            if (!worldBlockState.isAir) {
                distance = Vector3d.distance(
                    myPosCentered.x(),
                    myPosCentered.y(),
                    myPosCentered.z(),
                    searchBlockPos.x.toDouble(),
                    searchBlockPos.y.toDouble(),
                    searchBlockPos.z.toDouble()
                )
                result = true
            } else {
                val bounds = 0.5
                val searchAABB = AABB(
                    searchPos.x - bounds, searchPos.y - bounds, searchPos.z - bounds,
                    searchPos.x + bounds, searchPos.y + bounds, searchPos.z + bounds
                )
                val ships: Iterator<Ship> = level.getShipsIntersecting(searchAABB).iterator()
                var shipItr: Ship
                val transformedSearchPos: Vector3dc = Vector3d(searchPos)
                if (ships.hasNext()) {
                    do {
                        shipItr = ships.next()
                        if (shipItr === ship) continue
                        val transformedPos = shipItr.worldToShip.transformPosition(transformedSearchPos, Vector3d())
                        val blockPos: BlockPos = BlockPos.containing(transformedPos.toMinecraft())
                        if (level.isBlockInShipyard(blockPos)) {
                            val blockState: BlockState = level.getBlockState(blockPos)
                            if (!blockState.isAir && blockState.isFaceSturdy(
                                    level,
                                    blockPos,
                                    Direction.UP,
                                    SupportType.RIGID
                                )
                            ) {
                                searchBlockPos = BlockPos.containing(
                                    shipItr.shipToWorld.transformPosition(
                                        blockPos.x.toDouble(),
                                        blockPos.y.toDouble(),
                                        blockPos.z.toDouble(),
                                        Vector3d()
                                    ).toMinecraft()
                                )
                                distance = Vector3d.distance(
                                    myPosCentered.x(),
                                    myPosCentered.y(),
                                    myPosCentered.z(),
                                    searchBlockPos.x.toDouble(),
                                    searchBlockPos.y.toDouble(),
                                    searchBlockPos.z.toDouble()
                                )
                                result = true
                                ship2 = shipItr
                            }
                        }
                    } while (ships.hasNext() && !result)
                }
            }
            if (result && !level.isClientSide && attach) doAttach(
                level as ServerLevel,
                ship,
                ship2,
                myPosCentered,
                myDirNormal,
                compoundTag,
                distance
            )
            return result
        }

        fun doAttach(level: ServerLevel, ship1: Ship?, ship2: Ship?, myPos: Vector3dc, myDirNormal: Vector3dc, tag: CompoundTag, distance: Double) {
            if (ship1 == null && ship2 == null) return

            val adjustedDirNormal = Vector3d(myDirNormal).normalize().mul(1.0, Vector3d())
            val ship1Pos = Vector3d(myPos).add(adjustedDirNormal, Vector3d())
            val ship2ConstraintPos = Vector3d(ship1Pos)
            val ship2Pos: Vector3d? = null
            val ship1Rot: Quaterniond? = null
            val ship2Rot: Quaterniond? = null

            var adjustedDistance = distance
            var realShip1 = ship1
            var realShip2 = ship2
            var realShip1Pos = ship1Pos
            var realShip2Pos = ship2Pos
            var realShip1Rot = ship1Rot
            var realShip2Rot = ship2Rot
            var persistentKey = getAttachmentPersistentKey(tag)

            if (tag.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID) &&
                tag.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)
            ) {
                val attachConstraintData = tag.getByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)
                val attachConstraint = mapper.readValue(attachConstraintData, VSFixedJoint::class.java)
                var attachConstraintId =
                    level.gtpa.resolveRuntimeJointId(tag.getInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID))

                if (level.gtpa.getJointById(attachConstraintId) == null && persistentKey != null) {
                    val reboundJointId = level.gtpa.getRuntimeIdForPersistentKey(persistentKey)
                        ?.let(level.gtpa::resolveRuntimeJointId)
                    if (reboundJointId != null && level.gtpa.getJointById(reboundJointId) != null) {
                        attachConstraintId = reboundJointId
                    }
                }

                if (level.gtpa.getJointById(attachConstraintId) != null) {
                    tag.putInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID, attachConstraintId)
                    if (persistentKey == null) {
                        persistentKey = level.gtpa.getPersistentKeyForRuntimeId(attachConstraintId)
                        if (persistentKey != null) {
                            tag.putString(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY, persistentKey)
                        }
                    }

                    adjustedDistance = 1.0
                    realShip1 = attachConstraint.shipId0?.let { level.shipObjectWorld.loadedShips.getById(it) }
                    realShip2 = attachConstraint.shipId1?.let { level.shipObjectWorld.loadedShips.getById(it) }
                    realShip1Pos = Vector3d(attachConstraint.pose0.pos)
                    realShip2Pos = Vector3d(attachConstraint.pose1.pos)
                    realShip1Rot = Quaterniond(attachConstraint.pose0.rot)
                    realShip2Rot = Quaterniond(attachConstraint.pose1.rot)
                } else {
                    tag.remove(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID)
                    tag.remove(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)
                    tag.putBoolean(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ADD_QUEUED, false)
                }
            }

            val attachConstraint = makeConstraint(
                realShip1Pos,
                ship2ConstraintPos,
                realShip1,
                realShip2,
                level,
                realShip1Rot,
                realShip2Rot,
                realShip2Pos
            ) ?: return

            if (tag.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID)) {
                val runtimeJointId =
                    level.gtpa.resolveRuntimeJointId(tag.getInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID))
                if (level.gtpa.getJointById(runtimeJointId) != null) {
                    val previousConstraint = if (tag.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)) {
                        mapper.readValue(
                            tag.getByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT),
                            VSFixedJoint::class.java
                        )
                    } else {
                        null
                    }
                    if (previousConstraint == null || !jointsEquivalent(previousConstraint, attachConstraint)) {
                        level.gtpa.updateJointPersistent(runtimeJointId, attachConstraint)
                    }
                    tag.putInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID, runtimeJointId)
                    tag.putByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT, mapper.writeValueAsBytes(attachConstraint))
                    tag.putDouble(ClockworkConstants.Nbt.SHIP_SLICKER_DISTANCE, adjustedDistance)
                    tag.putBoolean(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ADD_QUEUED, false)
                    return
                }
            }

            if (persistentKey.isNullOrBlank()) {
                persistentKey = newPersistentJointKey()
            }
            val stablePersistentKey = persistentKey!!
            tag.putString(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY, stablePersistentKey)
            tag.putByteArray(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT, mapper.writeValueAsBytes(attachConstraint))
            tag.putDouble(ClockworkConstants.Nbt.SHIP_SLICKER_DISTANCE, adjustedDistance)

            if (tag.getBoolean(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ADD_QUEUED)) {
                return
            }
            tag.putBoolean(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ADD_QUEUED, true)

            level.gtpa.addJointPersistent(
                joint = attachConstraint,
                ownerType = "clockwork_slicker",
                ownerRef = buildPersistentKeyOwnerRef(stablePersistentKey),
                persistentKey = stablePersistentKey,
                delay = 3,
                function = { id ->
                    tag.putBoolean(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ADD_QUEUED, false)
                    if (id < 0) {
                        return@addJointPersistent
                    }
                    val resolvedId = level.gtpa.resolveRuntimeJointId(id)
                    tag.putInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID, resolvedId)
                    tag.putString(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY, stablePersistentKey)
                }
            )

            ClockworkMod.LOGGER.info(
                "Attached to ship ${attachConstraint.shipId1} using points ${attachConstraint.pose0.pos} && ${attachConstraint.pose1.pos}"
            )
        }

        private fun makeConstraint(
            ship1ConstraintPos: Vector3d,
            ship2ConstraintPos: Vector3d,
            ship1: Ship?,
            ship2: Ship?,
            level: ServerLevel,
            ship1Rot: Quaterniond?,
            ship2Rot: Quaterniond?,
            ship2Pos: Vector3d?
        ): VSFixedJoint? {
            if (ship1 == null && ship2 == null) return null

            val groundId: Long = level.shipObjectWorld.dimensionToGroundBodyIdImmutable[level.dimensionId]!!
            val ship1Id = ship1?.id ?: groundId
            val ship2Id = ship2?.id ?: groundId
            val ship1FrameId = ship1?.id
            val ship2FrameId = ship2?.id

            val pose0Pos = Vector3d(ship1ConstraintPos)
            val pose1Pos: Vector3dc = when {
                ship2Pos != null -> Vector3d(ship2Pos)
                else -> convertPoint(level, ship1FrameId, ship2FrameId, pose0Pos)
            }

            val ship1Rotation: Quaterniondc = ship1Rot?.let { Quaterniond(it).normalize() } ?: Quaterniond()
            val ship2Rotation: Quaterniondc = ship2Rot?.let { Quaterniond(it).normalize() } ?: Quaterniond()

            val attachConstraint = VSFixedJoint(
                ship1Id,
                pose0 = VSJointPose(pose0Pos, ship1Rotation),
                ship2Id,
                pose1 = VSJointPose(pose1Pos, ship2Rotation),
                VSJointMaxForceTorque(1.0E10F, 1.0E10F)
            )

            return attachConstraint
        }

        private fun getAttachmentPersistentKey(tag: CompoundTag): String? {
            return if (tag.contains(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY)) {
                tag.getString(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY).ifBlank { null }
            } else {
                null
            }
        }

        private fun jointsEquivalent(previous: VSFixedJoint, next: VSFixedJoint): Boolean {
            if (previous.shipId0 != next.shipId0 || previous.shipId1 != next.shipId1) {
                return false
            }
            val positionDelta = Vector3d(previous.pose0.pos).sub(next.pose0.pos).length() +
                Vector3d(previous.pose1.pos).sub(next.pose1.pos).length()
            if (positionDelta > JOINT_UPDATE_EPSILON) {
                return false
            }
            val rotDelta0 = 1.0 - abs(Quaterniond(previous.pose0.rot).dot(next.pose0.rot))
            if (rotDelta0 > JOINT_UPDATE_EPSILON) {
                return false
            }
            val rotDelta1 = 1.0 - abs(Quaterniond(previous.pose1.rot).dot(next.pose1.rot))
            return rotDelta1 <= JOINT_UPDATE_EPSILON
        }


        fun removeConstraint(level: ServerLevel, removeTags: Boolean, compoundTag: CompoundTag) {
            if (compoundTag.contains(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID)) {
                val runtimeJointId =
                    level.gtpa.resolveRuntimeJointId(compoundTag.getInt(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID))
                level.gtpa.removeJointPersistent(runtimeJointId)
            }

            if (removeTags) {
                compoundTag.remove(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ID)
                compoundTag.remove(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT)
                compoundTag.remove(ClockworkConstants.Nbt.ATTACHMENT_PERSISTENT_KEY)
                compoundTag.remove(ClockworkConstants.Nbt.ATTACHMENT_CONSTRAINT_ADD_QUEUED)
            }
        }
    }
}
