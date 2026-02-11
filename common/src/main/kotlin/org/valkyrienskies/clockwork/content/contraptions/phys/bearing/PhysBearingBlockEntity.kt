package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import com.simibubi.create.AllSoundEvents
import com.simibubi.create.content.contraptions.AbstractContraptionEntity
import com.simibubi.create.content.contraptions.AssemblyException
import com.simibubi.create.content.contraptions.ControlledContraptionEntity
import com.simibubi.create.content.contraptions.IDisplayAssemblyExceptions
import com.simibubi.create.content.contraptions.bearing.BearingBlock
import com.simibubi.create.content.contraptions.bearing.IBearingBlockEntity
import com.simibubi.create.content.kinetics.base.GeneratingKineticBlockEntity
import com.simibubi.create.content.kinetics.transmission.sequencer.SequencerInstructions
import com.simibubi.create.foundation.blockEntity.behaviour.BlockEntityBehaviour
import com.simibubi.create.foundation.blockEntity.behaviour.scrollValue.ScrollOptionBehaviour
import com.simibubi.create.foundation.item.TooltipHelper
import com.simibubi.create.foundation.utility.ServerSpeedProvider
import net.createmod.catnip.math.AngleHelper
import net.minecraft.core.BlockPos
import net.minecraft.core.Direction
import net.minecraft.nbt.CompoundTag
import net.minecraft.network.chat.Component
import net.minecraft.server.level.ServerLevel
import net.minecraft.util.Mth
import net.minecraft.world.level.ClipContext
import net.minecraft.world.level.block.entity.BlockEntityType
import net.minecraft.world.level.block.state.BlockState
import org.joml.*
import org.valkyrienskies.clockwork.ClockworkMod
import org.valkyrienskies.clockwork.ClockworkMod.MOD_ID
import org.valkyrienskies.clockwork.ClockworkSounds
import org.valkyrienskies.clockwork.content.contraptions.phys.bearing.data.PhysBearingData
import org.valkyrienskies.clockwork.content.contraptions.phys.bearing.data.PhysBearingUpdateData
import org.valkyrienskies.clockwork.content.forces.contraption.BearingController
import org.valkyrienskies.clockwork.content.forces.contraption.BearingController.Companion.getAngle
import org.valkyrienskies.clockwork.platform.api.ContraptionController
import org.valkyrienskies.clockwork.platform.api.ContraptionController.LockedMode
import org.valkyrienskies.clockwork.util.ClockworkConstants
import org.valkyrienskies.clockwork.util.ClockworkConstants.Nbt.ORIGINAL_DIRECTION
import org.valkyrienskies.clockwork.util.ClockworkUtils.getVector3d
import org.valkyrienskies.clockwork.util.GlueAssembler.collectGlued
import org.valkyrienskies.clockwork.util.gtpa
import org.valkyrienskies.clockwork.util.updateJoint
import org.valkyrienskies.clockwork.util.minus
import org.valkyrienskies.clockwork.util.plus
import org.valkyrienskies.clockwork.util.times
import org.valkyrienskies.core.api.attachment.getAttachment
import org.valkyrienskies.core.api.ships.PhysShip
import org.valkyrienskies.core.api.ships.ServerShip
import org.valkyrienskies.core.api.world.PhysLevel
import org.valkyrienskies.core.api.world.properties.DimensionId
import org.valkyrienskies.core.internal.joints.*
import org.valkyrienskies.core.impl.bodies.properties.BodyTransformFactory

import org.valkyrienskies.core.impl.util.serialization.VSJacksonUtil
import org.valkyrienskies.core.internal.world.VsiPhysLevel
import org.valkyrienskies.core.util.datastructures.DenseBlockPosSet
import org.valkyrienskies.kelvin.util.KelvinExtensions.toMinecraft
import org.valkyrienskies.kelvin.util.KelvinExtensions.toVector3d
import org.valkyrienskies.mod.api.BlockEntityPhysicsListener
import org.valkyrienskies.mod.api.dimensionId
import org.valkyrienskies.mod.common.*
import org.valkyrienskies.mod.common.assembly.ShipAssembler.assembleToShip
import org.valkyrienskies.mod.common.assembly.VSAssemblyEvents
import org.valkyrienskies.mod.common.util.SplittingDisablerAttachment
import org.valkyrienskies.mod.common.util.toJOML
import org.valkyrienskies.mod.common.util.toJOMLD
import org.valkyrienskies.mod.common.world.clipIncludeShips
import org.valkyrienskies.mod.util.putVector3d
import java.lang.Math
import kotlin.math.*

//TODO move
fun getHingeRotation(localDir: Vector3dc, right: Vector3dc = Vector3d(1.0, 0.0, 0.0)): Quaterniond {
    if ((localDir - right).length() < 1e-5) { return Quaterniond() }

    val v1l = right.length()
    val v2l = localDir.length()

    val a = right.cross(localDir, Vector3d())

    val k = sqrt(v1l * v1l * v2l * v2l)
    val kCosTheta = right.dot(localDir)

    if (abs(kCosTheta / k + 1.0) < 1e-5) {
        val ort = right.let { it.orthogonalize(it, Vector3d()) }
        return Quaterniond(ort.x, ort.y, ort.z, 0.0).normalize()
    }

    return Quaterniond(a.x, a.y, a.z, k + kCosTheta).normalize()
}

class PhysBearingBlockEntity(type: BlockEntityType<*>?, pos: BlockPos?, state: BlockState?) :
    GeneratingKineticBlockEntity(type, pos, state), IBearingBlockEntity, IDisplayAssemblyExceptions,
    ContraptionController, BlockEntityPhysicsListener {

    var assembleNextTick = false
    var movementMode: ScrollOptionBehaviour<LockedMode>? = null
    var isRunning = false
        private set
    var shiptraptionID = NO_SHIPTRAPTION_ID
        private set
    var targetAngle = 0f
        get() = field
        private set(idk) {field = idk}
    var disassembleWhenPossible = false
        private set
    @Volatile var joint : VSJoint? = null
        private set
    @Volatile var jointID : Int = -1
        private set

    private var lastException: AssemblyException? = null
    private var open = false
    private var originalDirection: Direction? = null
    private var clientAngleDiff = 0f
    private var prevAngle = 0f
    private var coreAngle = 0f
    private var previousCoreAngle = 0f

    private var opening = false
    private var openProgress = 0f
    private var openProgressMax = 70f
    private var inOutCorner = 0f
    private var cornerShrinking = false

    private var ticks = 0
    private var lastStateChanged = 0
    private var cooldown = 20

    private var sequencedAngleLimit = -1.0f
    private var sequencedAngleProgress = 0f

    //pos of bearing in subship coordinates
    private var bearingPos: Vector3d = Vector3d()
    private var aligning = false
    private var bearingAxis: Vector3d = Vector3d()
    private var bearingID: Int = -1

    private var lastSpeed = 0f
    private var lastMode = LockedMode.UNLOCKED
    private var lastAligningState = false
    @Volatile private var servoMode: LockedMode = LockedMode.UNLOCKED
    @Volatile private var lockedHoldAngleRad: Double? = null
    @Volatile private var activeFixedTargetRad: Double? = null
    @Volatile private var fixedZeroRelRotMainToSub: Quaterniond? = null
    @Volatile private var fixedReferenceCaptured = false
    @Volatile private var fixedReferenceAngleRad = 0.0
    @Volatile private var fixedAnchorPose0Local: Vector3d? = null
    @Volatile private var fixedAnchorPose1Local: Vector3d? = null
    @Volatile private var fixedAnchorCaptured = false
    @Volatile private var fixedDesiredTargetRadContinuous: Double? = null
    @Volatile private var fixedDesiredTargetInitialized = false
    @Volatile private var fixedModeInitialized = false
    @Volatile private var fixedAcquireTicksRemaining = 0
    @Volatile private var fixedPostLoadSettleTicksRemaining = 0
    @Volatile private var needsJointRematerialization = false
    @Volatile private var lastAppliedJointKindFixed = false
    @Volatile private var lastSentFixedTargetRad: Double? = null
    @Volatile private var lastFixedRefreshTick: Int = -FIXED_TARGET_SAFETY_REFRESH_TICKS
    @Volatile private var fixedTrackStepRadPerTick = 0.0
    @Volatile private var fixedTrackStepAccelRadPerTick2 = 0.0
    @Volatile private var fixedTrackStepInitialized = false
    @Volatile private var fixedTrackLastMeasuredAngleRad: Double? = null
    @Volatile private var fixedWasMovingFollow = false
    @Volatile private var fixedStopHoldLatched = false
    @Volatile private var fixedStopHoldAngleRad: Double? = null
    @Volatile private var reconnectFreezeRequested = false
    @Volatile private var reconnectFreezeLeaseHeld = false
    @Volatile private var reconnectFreezeShipId = NO_SHIPTRAPTION_ID

    private var controllerCreationData: PhysBearingData? = null
    private var controllerUpdateData: PhysBearingUpdateData? = null
    private var loadingFn: ((ServerLevel) -> Boolean)? = null

    init {
        setLazyTickRate(3)
    }

    private fun movementModeChanged(value: Int) {
        if (level == null || level!!.isClientSide) {return}
        tryUpdateData()
        sendData()
    }

    override fun addBehaviours(behaviours: MutableList<BlockEntityBehaviour>) {
        super.addBehaviours(behaviours)
        movementMode = ScrollOptionBehaviour(
            LockedMode::class.java, Component.translatable("$MOD_ID.phys_bearing.rotation_mode"),
            this, movementModeSlot
        )
        movementMode!!.withCallback{movementModeChanged(it)}
        movementMode!!.requiresWrench()
        behaviours.add(movementMode!!)
    }

    private fun computeJointMaxForceTorque(): VSJointMaxForceTorque {
        return VSJointMaxForceTorque(SERVO_JOINT_FORCE_MAX.toFloat(), SERVO_JOINT_TORQUE_MAX.toFloat())
    }

    private fun shouldUseFixedJoint(mode: LockedMode, aligningNow: Boolean): Boolean {
        return PhysBearingFollowController.isFixedJointMode(mode.name, aligningNow)
    }

    private fun fixedTargetAngleRad(mode: LockedMode, aligningNow: Boolean): Double {
        return when {
            aligningNow -> 0.0
            mode == LockedMode.LOCKED -> lockedHoldAngleRad ?: Math.toRadians(targetAngle.toDouble())
            else -> Math.toRadians(targetAngle.toDouble())
        }
    }

    private fun shortestAngleErrorRad(target: Double, current: Double): Double {
        return PhysBearingFollowController.normalizeAngleErrorRad(target, current)
    }

    private fun currentJointKindIsFixed(): Boolean {
        return joint is VSFixedJoint
    }

    private fun currentJointAngleRad(level: ServerLevel, sourceJoint: VSJoint? = joint): Double? {
        val joint = sourceJoint ?: return null
        val subId = joint.shipId0 ?: return null
        val subShip = level.shipObjectWorld.loadedShips.getById(subId) ?: return null
        val mainShip = joint.shipId1?.let { level.shipObjectWorld.loadedShips.getById(it) }
        return getAngle(bearingAxis, subShip.transform, mainShip?.transform).takeIf { it.isFinite() }
    }

    private fun captureFixedZeroReference(level: ServerLevel, currentAngleRad: Double): Boolean {
        if (!currentAngleRad.isFinite()) return false
        val joint = joint ?: return false
        val subId = joint.shipId0 ?: return false
        val subShip = level.shipObjectWorld.loadedShips.getById(subId) ?: return false
        val mainShip = joint.shipId1?.let { level.shipObjectWorld.loadedShips.getById(it) }

        val subRotWorld = Quaterniond(subShip.transform.shipToWorldRotation)
        val mainRotWorld = if (mainShip != null) Quaterniond(mainShip.transform.shipToWorldRotation) else Quaterniond()
        if (!subRotWorld.isFiniteQuat() || !mainRotWorld.isFiniteQuat()) return false

        val relNowMainToSub = Quaterniond(mainRotWorld).conjugate().mul(subRotWorld, Quaterniond()).normalize()
        if (!relNowMainToSub.isFiniteQuat()) return false

        val axisMainLocal = bearingAxis.get(Vector3d())
        if (!axisMainLocal.isFiniteVec() || axisMainLocal.lengthSquared() < 1.0e-12) return false

        val zeroRef = PhysBearingFollowController.captureZeroRelRotation(
            relNowMainToSub,
            axisMainLocal,
            currentAngleRad
        )
        if (!zeroRef.isFiniteQuat()) return false

        fixedZeroRelRotMainToSub = zeroRef
        fixedReferenceCaptured = true
        fixedReferenceAngleRad = currentAngleRad
        return true
    }

    private fun captureFixedAnchorReference(level: ServerLevel): Boolean {
        val joint = joint ?: return false
        val subId = joint.shipId0 ?: return false
        val subShip = level.shipObjectWorld.loadedShips.getById(subId) ?: return false
        val mainShip = joint.shipId1?.let { level.shipObjectWorld.loadedShips.getById(it) }

        val anchorWorld = if (mainShip != null) {
            mainShip.transform.shipToWorld.transformPosition(joint.pose1.pos, Vector3d())
        } else {
            joint.pose1.pos.get(Vector3d())
        }
        if (!anchorWorld.isFiniteVec()) return false

        val pose0Local = subShip.transform.worldToShip.transformPosition(anchorWorld, Vector3d())
        if (!pose0Local.isFiniteVec()) return false
        val pose1Local = if (mainShip != null) {
            mainShip.transform.worldToShip.transformPosition(anchorWorld, Vector3d())
        } else {
            anchorWorld
        }
        if (!pose1Local.isFiniteVec()) return false

        fixedAnchorPose0Local = pose0Local
        fixedAnchorPose1Local = pose1Local
        fixedAnchorCaptured = true
        return true
    }

    private fun resetFixedModeRuntimeState() {
        activeFixedTargetRad = null
        fixedZeroRelRotMainToSub = null
        fixedReferenceCaptured = false
        fixedReferenceAngleRad = 0.0
        fixedAnchorPose0Local = null
        fixedAnchorPose1Local = null
        fixedAnchorCaptured = false
        fixedDesiredTargetRadContinuous = null
        fixedDesiredTargetInitialized = false
        fixedModeInitialized = false
        fixedAcquireTicksRemaining = 0
        fixedPostLoadSettleTicksRemaining = 0
        needsJointRematerialization = false
        lastAppliedJointKindFixed = false
        lastSentFixedTargetRad = null
        lastFixedRefreshTick = -FIXED_TARGET_SAFETY_REFRESH_TICKS
        fixedTrackStepRadPerTick = 0.0
        fixedTrackStepAccelRadPerTick2 = 0.0
        fixedTrackStepInitialized = false
        fixedTrackLastMeasuredAngleRad = null
        fixedWasMovingFollow = false
        fixedStopHoldLatched = false
        fixedStopHoldAngleRad = null
    }

    private fun requestReconnectFreeze(shipId: Long = shiptraptionID) {
        if (shipId == NO_SHIPTRAPTION_ID) return
        reconnectFreezeRequested = true
        reconnectFreezeShipId = shipId
    }

    private fun holdReconnectFreeze(level: ServerLevel, shipId: Long = reconnectFreezeShipId): Boolean {
        if (!reconnectFreezeRequested || shipId == NO_SHIPTRAPTION_ID) return false
        val ship = level.shipObjectWorld.loadedShips.getById(shipId) ?: return false
        if (!reconnectFreezeLeaseHeld || reconnectFreezeShipId != shipId) {
            if (reconnectFreezeLeaseHeld) {
                releaseReconnectFreeze(level)
            }
            acquireReconnectFreezeLease(shipId, ship.isStatic)
            reconnectFreezeLeaseHeld = true
            reconnectFreezeShipId = shipId
        }
        ship.isStatic = true
        return true
    }

    private fun holdReconnectFreeze(level: VsiPhysLevel, shipId: Long): Boolean {
        if (!reconnectFreezeRequested || shipId == NO_SHIPTRAPTION_ID) return false
        val ship = level.getShipById(shipId) ?: return false
        if (!reconnectFreezeLeaseHeld || reconnectFreezeShipId != shipId) {
            if (reconnectFreezeLeaseHeld) {
                releaseReconnectFreeze(level)
            }
            acquireReconnectFreezeLease(shipId, ship.isStatic)
            reconnectFreezeLeaseHeld = true
            reconnectFreezeShipId = shipId
        }
        ship.isStatic = true
        return true
    }

    private fun releaseReconnectFreeze(level: ServerLevel?) {
        val shipId = reconnectFreezeShipId
        val leaseHeld = reconnectFreezeLeaseHeld

        reconnectFreezeRequested = false
        reconnectFreezeLeaseHeld = false
        reconnectFreezeShipId = NO_SHIPTRAPTION_ID

        if (!leaseHeld || shipId == NO_SHIPTRAPTION_ID) return
        val restoreStatic = releaseReconnectFreezeLease(shipId) ?: return
        if (level == null) return
        level.shipObjectWorld.loadedShips.getById(shipId)?.isStatic = restoreStatic
    }

    private fun releaseReconnectFreeze(level: VsiPhysLevel) {
        val shipId = reconnectFreezeShipId
        val leaseHeld = reconnectFreezeLeaseHeld

        reconnectFreezeRequested = false
        reconnectFreezeLeaseHeld = false
        reconnectFreezeShipId = NO_SHIPTRAPTION_ID

        if (!leaseHeld || shipId == NO_SHIPTRAPTION_ID) return
        val restoreStatic = releaseReconnectFreezeLease(shipId) ?: return
        level.getShipById(shipId)?.isStatic = restoreStatic
    }

    private fun Quaterniondc.isFiniteQuat(): Boolean {
        return x().isFinite() && y().isFinite() && z().isFinite() && w().isFinite()
    }

    private fun Vector3dc.isFiniteVec(): Boolean {
        return x().isFinite() && y().isFinite() && z().isFinite()
    }

    private fun toRevoluteJoint(joint: VSJoint, maxForceTorque: VSJointMaxForceTorque = computeJointMaxForceTorque()): VSRevoluteJoint {
        return when (joint) {
            is VSRevoluteJoint -> joint.copy(
                maxForceTorque = maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                driveVelocity = null,
                driveForceLimit = null,
                driveGearRatio = null,
                driveFreeSpin = true
            )
            is VSFixedJoint -> VSRevoluteJoint(
                joint.shipId0,
                joint.pose0,
                joint.shipId1,
                joint.pose1,
                maxForceTorque = maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                driveFreeSpin = true
            )
            else -> throw IllegalArgumentException("Unsupported joint type for revolute conversion: ${joint::class.java.simpleName}")
        }
    }

    private fun materializeFixedTargetJoint(
        joint: VSJoint,
        targetAngleRad: Double,
        level: ServerLevel,
        maxForceTorque: VSJointMaxForceTorque = computeJointMaxForceTorque()
    ): VSFixedJoint? {
        val subId = joint.shipId0 ?: return null
        val subShip = level.shipObjectWorld.loadedShips.getById(subId)
            ?: return null
        val mainShip = joint.shipId1?.let { level.shipObjectWorld.loadedShips.getById(it) }
        val zeroRef = fixedZeroRelRotMainToSub ?: return null
        val pose0Local = fixedAnchorPose0Local ?: return null
        val pose1Local = fixedAnchorPose1Local ?: return null
        if (!fixedAnchorCaptured) return null
        if (!targetAngleRad.isFinite()) return null

        val mainRotWorld = if (mainShip != null) Quaterniond(mainShip.transform.shipToWorldRotation) else Quaterniond()
        if (!mainRotWorld.isFiniteQuat()) return null

        val axisMainLocal = bearingAxis.get(Vector3d())
        if (!axisMainLocal.isFiniteVec() || axisMainLocal.lengthSquared() < 1.0e-12) return null

        val desiredRelMainToSub = PhysBearingFollowController.desiredRelRotationFromZeroRef(
            zeroRef,
            axisMainLocal,
            PhysBearingFollowController.wrapAngleSignedPiRad(targetAngleRad)
        )
        if (!desiredRelMainToSub.isFiniteQuat()) return null
        val desiredSubRotWorld = Quaterniond(mainRotWorld).mul(desiredRelMainToSub, Quaterniond()).normalize()
        if (!desiredSubRotWorld.isFiniteQuat()) return null

        val sharedFrameWorld = if (mainShip != null) {
            Quaterniond(mainRotWorld).mul(joint.pose1.rot, Quaterniond())
        } else {
            Quaterniond(joint.pose1.rot)
        }.normalize()
        if (!sharedFrameWorld.isFiniteQuat()) return null

        val pose0RotRaw = Quaterniond(desiredSubRotWorld)
            .conjugate()
            .mul(sharedFrameWorld, Quaterniond())
            .normalize()
        val pose0Rot = PhysBearingFollowController.enforceQuaternionHemisphere(pose0RotRaw, joint.pose0.rot)
        if (!pose0Rot.isFiniteQuat()) return null

        return VSFixedJoint(
            joint.shipId0,
            VSJointPose(pose0Local, pose0Rot),
            joint.shipId1,
            VSJointPose(pose1Local, joint.pose1.rot),
            maxForceTorque
        )
    }

    private fun updateDrive(
        forcedFixedTargetRad: Double? = null,
        forcedAuthority: VSJointMaxForceTorque? = null
    ): Boolean {
        val level = level as? ServerLevel ?: return false
        val existing = joint ?: return false
        val mode = movementMode?.get() ?: LockedMode.UNLOCKED
        val useFixed = shouldUseFixedJoint(mode, aligning)
        val maxForceTorque = forcedAuthority ?: computeJointMaxForceTorque()
        val newJoint: VSJoint = if (useFixed) {
            if (!fixedReferenceCaptured || fixedZeroRelRotMainToSub == null) return false
            if (!fixedAnchorCaptured || fixedAnchorPose0Local == null || fixedAnchorPose1Local == null) return false
            val targetRad = forcedFixedTargetRad ?: activeFixedTargetRad ?: fixedTargetAngleRad(mode, aligning)
            val fixed = materializeFixedTargetJoint(existing, targetRad, level, maxForceTorque) ?: return false
            lastSentFixedTargetRad = targetRad
            lastFixedRefreshTick = ticks
            fixed
        } else {
            toRevoluteJoint(existing, maxForceTorque).copy(
                maxForceTorque = maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                driveVelocity = null,
                driveForceLimit = null,
                driveGearRatio = null,
                driveFreeSpin = true
            )
        }

        joint = newJoint
        servoMode = mode
        lastAppliedJointKindFixed = useFixed
        needsJointRematerialization = false
        controllerUpdateData = PhysBearingUpdateData(
            Math.toRadians(targetAngle.toDouble()),
            if (mode == LockedMode.UNLOCKED && !aligning) getRealisticAngularSpeed() else 0f,
            useFixed
        )

        if (jointID != -1) {
            level.gtpa.updateJoint(jointID, newJoint)
        }
        return true
    }

    @Volatile override lateinit var dimension: DimensionId
    @Volatile private var lastAngle = targetAngle
    @Volatile private var curAngle = targetAngle
    override fun physTick(physShip: PhysShip?, physLevel: PhysLevel) {
        if (isRemoved || !isRunning) return
        val current = joint ?: return
        val expectedFixed = shouldUseFixedJoint(servoMode, aligning)

        if ((expectedFixed && current !is VSFixedJoint) || (!expectedFixed && current !is VSRevoluteJoint)) {
            needsJointRematerialization = true
        }
    }

    public override fun write(tag: CompoundTag, clientPacket: Boolean) {
        super.write(tag, clientPacket)

        tag.putBoolean(ClockworkConstants.Nbt.RUNNING, isRunning)
        tag.putFloat(ClockworkConstants.Nbt.ANGLE, targetAngle)
        if (shiptraptionID != NO_SHIPTRAPTION_ID) {
            tag.putLong(ClockworkConstants.Nbt.SHIPTRAPTION_ID, shiptraptionID)
        }
        if (originalDirection != null) {
            tag.putInt(ORIGINAL_DIRECTION, originalDirection!!.ordinal)
        }
        AssemblyException.write(tag, lastException)
        tag.putBoolean(ClockworkConstants.Nbt.OPEN, open)
        tag.putFloat(ClockworkConstants.Nbt.SEQUENCED_ANGLE_LIMIT, sequencedAngleLimit)
        tag.putFloat(ClockworkConstants.Nbt.SEQUENCED_ANGLE_PROGRESS, sequencedAngleProgress)

        tag.putVector3d("bearingPos", bearingPos)
        tag.putVector3d("bearingAxis", bearingAxis)
        tag.putBoolean("aligning", aligning)

        val mapper = VSJacksonUtil.dtoMapper
        val jointForSave: VSRevoluteJoint? = when (val runtimeJoint = joint) {
            is VSRevoluteJoint -> runtimeJoint.copy(driveVelocity = null)
            is VSFixedJoint -> VSRevoluteJoint(
                runtimeJoint.shipId0,
                runtimeJoint.pose0,
                runtimeJoint.shipId1,
                runtimeJoint.pose1,
                maxForceTorque = runtimeJoint.maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                driveFreeSpin = true
            )
            else -> null
        }
        if (jointForSave != null) {
            tag.putByteArray("joint", mapper.writeValueAsBytes(jointForSave))
        }

        tag.putInt("jointID", jointID)

        if (shiptraptionID == NO_SHIPTRAPTION_ID) return

        tag.putLong(ClockworkConstants.Nbt.OLD_POS, worldPosition.asLong())
        //to make it more general
        tag.putVector3d(ClockworkConstants.Nbt.OLD_SHIPTRAPTION_CENTER, bearingPos)
        tag.putVector3d(ClockworkConstants.Nbt.NEW_SHIPTRAPTION_CENTER, bearingPos)
    }

    private fun loadTheRest(tag: CompoundTag, level: ServerLevel): Boolean {
        val savedJoint = this.joint ?: return false
        if (isRunning) {
            requestReconnectFreeze(shiptraptionID)
            holdReconnectFreeze(level, shiptraptionID)
        }

        val savedMainId = savedJoint.shipId1
        val resolvedMainId = level.getShipManagingPos(worldPosition)?.id
        val loadResolution = PhysBearingLoadController.resolveMainIdForLoad(savedMainId, resolvedMainId)
        if (loadResolution.defer) {
            return false
        }
        val mainIdForJoint = loadResolution.mainIdForJoint
        val subLoaded = level.shipObjectWorld.loadedShips.getById(shiptraptionID) != null
        val mainLoaded = mainIdForJoint?.let { level.shipObjectWorld.loadedShips.getById(it) != null } ?: true
        if (PhysBearingLoadController.shouldDeferForEndpointAvailability(subLoaded, mainIdForJoint, mainLoaded)) {
            return false
        }

        val oldSPos = tag.getVector3d(ClockworkConstants.Nbt.OLD_SHIPTRAPTION_CENTER) ?: return false
        val newSPos = tag.getVector3d(ClockworkConstants.Nbt.NEW_SHIPTRAPTION_CENTER) ?: return false
        val oldBPos = BlockPos.of(tag.getLong(ClockworkConstants.Nbt.OLD_POS))
        val oldPos = oldBPos.toJOMLD()
        val newPos = worldPosition.toJOMLD()

        val savedBearingPos = tag.getVector3d("bearingPos") ?: return false
        bearingPos = savedBearingPos.sub(oldSPos, Vector3d()).add(newSPos)

        this.joint = when (savedJoint) {
            is VSRevoluteJoint -> savedJoint.copy(
                shiptraptionID,
                pose0 = VSJointPose(savedJoint.pose0.pos - oldSPos + newSPos, savedJoint.pose0.rot),
                mainIdForJoint,
                pose1 = VSJointPose(savedJoint.pose1.pos - oldPos + newPos, savedJoint.pose1.rot)
            )
            is VSFixedJoint -> savedJoint.copy(
                shiptraptionID,
                pose0 = VSJointPose(savedJoint.pose0.pos - oldSPos + newSPos, savedJoint.pose0.rot),
                mainIdForJoint,
                pose1 = VSJointPose(savedJoint.pose1.pos - oldPos + newPos, savedJoint.pose1.rot)
            )
            else -> throw AssertionError()
        }

        controllerCreationData = PhysBearingData(
            bearingAxis.get(Vector3d()),
            Math.toRadians(targetAngle.toDouble()),
            getRealisticAngularSpeed(),
            shouldUseFixedJoint(movementMode?.get() ?: LockedMode.UNLOCKED, aligning),
            aligning,
            mainIdForJoint ?: -1L,
            this.joint?.pose1?.pos?.get(Vector3d()) ?: Vector3d(),
            this.joint?.pose0?.pos?.get(Vector3d()) ?: Vector3d()
        )

        resetFixedModeRuntimeState()
        fixedAcquireTicksRemaining = FIXED_ACQUIRE_TICKS
        fixedPostLoadSettleTicksRemaining = FIXED_POST_LOAD_SETTLE_TICKS
        needsJointRematerialization = true
        tryMakeJoint()
        tryUpdateData()
        return true
    }

    override fun read(tag: CompoundTag, clientPacket: Boolean) {
        if (wasMoved) {
            super.read(tag, clientPacket)
            return
        }
        val angleBefore = targetAngle
        open = tag.getBoolean(ClockworkConstants.Nbt.OPEN)
        isRunning = tag.getBoolean(ClockworkConstants.Nbt.RUNNING)
        targetAngle = tag.getFloat(ClockworkConstants.Nbt.ANGLE)
        lastAngle = targetAngle
        curAngle = targetAngle
        lastException = AssemblyException.read(tag)
        if (tag.contains(ClockworkConstants.Nbt.SHIPTRAPTION_ID)) {
            shiptraptionID = tag.getLong(ClockworkConstants.Nbt.SHIPTRAPTION_ID)
        }
        if (tag.contains(ORIGINAL_DIRECTION)) {
            originalDirection = Direction.entries[tag.getInt(ORIGINAL_DIRECTION)]
        }
        if (isRunning) {
            if (shiptraptionID == NO_SHIPTRAPTION_ID) {
                clientAngleDiff = AngleHelper.getShortestAngleDiff(angleBefore.toDouble(), targetAngle.toDouble())
                targetAngle = angleBefore
            }
        } else {
            shiptraptionID = NO_SHIPTRAPTION_ID
        }
        sequencedAngleLimit = tag.getFloat(ClockworkConstants.Nbt.SEQUENCED_ANGLE_LIMIT)
        sequencedAngleProgress = tag.getFloat(ClockworkConstants.Nbt.SEQUENCED_ANGLE_PROGRESS)

        bearingPos = tag.getVector3d("bearingPos")!!
        bearingAxis = tag.getVector3d("bearingAxis")!!
        aligning = tag.getBoolean("aligning")

        val mapper = VSJacksonUtil.dtoMapper

        if (tag.contains("constraint")) {
            val temp = mapper.readValue(tag.getByteArray("constraint"), VSJointAndId::class.java)
            joint = temp.joint
            jointID = temp.jointId
        } else if (tag.contains("joint")) {
            joint = mapper.readValue(tag.getByteArray("joint"), VSRevoluteJoint::class.java).copy(driveVelocity = null)
            jointID = tag.getInt("jointID")
        } else if (tag.contains("fjoint")) {
            joint = mapper.readValue(tag.getByteArray("fjoint"), VSFixedJoint::class.java)
            jointID = tag.getInt("jointID")
        }

        super.read(tag, clientPacket)
        if (clientPacket) {return}

        // may not have level on read so i have to do this
        loadingFn = { level -> loadTheRest(tag, level) }

        val level = level as? ServerLevel ?: return
        if (loadingFn?.invoke(level) == true) {
            loadingFn = null
        }
    }

    override fun getInterpolatedAngle(partialTicks: Float): Float {
        var partialTicks = partialTicks
        if (isVirtual) return Mth.lerp(partialTicks + .5f, prevAngle, targetAngle)
        if (shiptraptionID == NO_SHIPTRAPTION_ID || !isRunning) partialTicks = 0f
        return Mth.lerp(partialTicks, targetAngle, targetAngle + angularSpeed)
    }

    fun getWingRotOffset(): Float = when {
         isRunning && open -> openProgressMax.toDouble().toFloat()
         isRunning         -> Mth.lerp(openProgress.toDouble(), 0.0, openProgressMax.toDouble()).toFloat()
        !isRunning && open -> Mth.lerp(openProgress.toDouble(), 1.0, openProgressMax.toDouble()).toFloat()
        else -> 0.0f
    }

    fun getInterpolatedCoreAngle(partialTicks: Float): Float {
        previousCoreAngle = coreAngle
        coreAngle++
        if (coreAngle == 360f) {
            coreAngle = 0f
        }
        return if (isVirtual) Mth.lerp(partialTicks + .5f, previousCoreAngle, coreAngle) else Mth.lerp(
            partialTicks,
            coreAngle,
            coreAngle + 4f
        )
    }

    val angularSpeed: Float
        get() {
            val mode = movementMode?.get() ?: LockedMode.UNLOCKED
            if (aligning || mode == LockedMode.LOCKED) return 0f
            var speed = convertToAngular(getSpeed())
            if (getSpeed() == 0f) speed = 0f
            if (level!!.isClientSide) {
                speed *= ServerSpeedProvider.get()
                speed += clientAngleDiff / 3f
            }
            return speed
        }

    private fun getHingeRotation(localDirection: Direction): Quaterniond {
        val rotationQuaternion: Quaterniond = when (localDirection) {
            Direction.UP -> {
                Quaterniond()
            }
            Direction.DOWN -> {
                Quaterniond(AxisAngle4d(Math.PI, Vector3d(1.0, 0.0, 0.0)))
            }
            Direction.NORTH -> {
                Quaterniond(AxisAngle4d(Math.PI, Vector3d(0.0, 1.0, 0.0))).mul(
                    Quaterniond(
                        AxisAngle4d(
                            Math.PI / 2.0, Vector3d(1.0, 0.0, 0.0)
                        )
                    )
                ).normalize()
            }
            Direction.EAST -> {
                Quaterniond(AxisAngle4d(0.5 * Math.PI, Vector3d(0.0, 1.0, 0.0))).mul(
                    Quaterniond(
                        AxisAngle4d(
                            Math.PI / 2.0, Vector3d(1.0, 0.0, 0.0)
                        )
                    )
                ).normalize()
            }
            Direction.SOUTH -> {
                Quaterniond(AxisAngle4d(Math.PI / 2.0, Vector3d(1.0, 0.0, 0.0))).normalize()
            }
            Direction.WEST -> {
                Quaterniond(AxisAngle4d(1.5 * Math.PI, Vector3d(0.0, 1.0, 0.0))).mul(
                    Quaterniond(
                        AxisAngle4d(
                            Math.PI / 2.0, Vector3d(1.0, 0.0, 0.0)
                        )
                    )
                ).normalize()
            }
        }

        val hingeOrientation: Quaterniond = rotationQuaternion.mul(
            Quaterniond(AxisAngle4d(Math.toRadians(90.0), 0.0, 0.0, 1.0)),
            Quaterniond()
        ).normalize()

        return hingeOrientation
    }

    fun tryMakeJoint() {
        ClockworkMod.physTickOnce(level.dimensionId!!) { level, _, tryNextTick ->
            level as VsiPhysLevel
            val initialJoint = this.joint ?: return@physTickOnce
            if (reconnectFreezeRequested) {
                val freezeShipId = reconnectFreezeShipId.takeIf { it != NO_SHIPTRAPTION_ID }
                    ?: initialJoint.shipId0
                    ?: shiptraptionID
                if (freezeShipId != NO_SHIPTRAPTION_ID) {
                    holdReconnectFreeze(level, freezeShipId)
                }
            }

            val availabilityJoint = this.joint ?: return@physTickOnce
            if (
                availabilityJoint.shipId0 != null && level.getShipById(availabilityJoint.shipId0!!) == null ||
                availabilityJoint.shipId1 != null && level.getShipById(availabilityJoint.shipId1!!) == null
            ) {
                tryNextTick()
                return@physTickOnce
            }

            val existing = level.getJointById(jointID)
            if (existing != null) {
                val updateJoint = this.joint ?: return@physTickOnce
                val sameKindAndEndpoints =
                    (existing is VSRevoluteJoint && updateJoint is VSRevoluteJoint &&
                        existing.shipId0 == updateJoint.shipId0 && existing.shipId1 == updateJoint.shipId1) ||
                        (existing is VSFixedJoint && updateJoint is VSFixedJoint &&
                            existing.shipId0 == updateJoint.shipId0 && existing.shipId1 == updateJoint.shipId1)
                if (sameKindAndEndpoints) {
                    level.updateJoint(jointID, updateJoint)
                    isRunning = true
                    if (reconnectFreezeRequested) {
                        releaseReconnectFreeze(level)
                    }
                    return@physTickOnce
                }
                if (existing == updateJoint) {
                    isRunning = true
                    if (reconnectFreezeRequested) {
                        releaseReconnectFreeze(level)
                    }
                    return@physTickOnce
                }
                level.removeJoint(jointID)
                jointID = -1
            }

            val addJoint = this.joint ?: return@physTickOnce
            val id = level.addJoint(addJoint)
            if (id == -1) {
                tryNextTick()
                return@physTickOnce
            }
            this.jointID = id

            isRunning = true
            if (reconnectFreezeRequested) {
                releaseReconnectFreeze(level)
            }
            lastStateChanged = ticks
        }
    }

    private fun assemble() {
        if (level!!.getBlockState(worldPosition).block !is BearingBlock) return
        val level = level as ServerLevel

        originalDirection = blockState.getValue(BearingBlock.FACING)
        val direction = originalDirection!!
        val attachPoint = worldPosition.relative(direction)

        // bearing data
        val worldPos: Vector3dc = worldPosition.center.toJOML()
        val axis = direction.normal.toJOMLD()
        val shipOn = level.getShipObjectManagingPos(worldPosition)

        val startPos = worldPos + axis * 0.5
        val endPos = worldPos + axis * 1.5

        fun Vector3d.toVec3() = this.let { net.minecraft.world.phys.Vec3(it.x, it.y, it.z) }

        val otherPos = level.clipIncludeShips(
            ClipContext(
                (shipOn?.transform?.shipToWorld?.transformPosition(startPos) ?: startPos).toVec3(),
                (shipOn?.transform?.shipToWorld?.transformPosition(endPos) ?: endPos).toVec3(),
                ClipContext.Block.COLLIDER,
                ClipContext.Fluid.NONE,
                null
                ), false, shipOn?.id)

        val otherShip = level.getShipObjectManagingPos(otherPos.blockPos)
        val posInOwnerShip = Vector3d(worldPos)

        val (bearingPos, shiptraption, otherDirection) = if (otherShip == null) {
            val selection: DenseBlockPosSet?
            try {
                selection = collectGlued(level, attachPoint)
                selection?.remove(this.blockPos.x, this.blockPos.y, this.blockPos.z)
                lastException = null
            } catch (e: AssemblyException) {
                lastException = e
                sendData()
                return
            }
            if (selection == null) return

            var centerPositions: Pair<Vector3d, Vector3d> = Pair(Vector3d(), Vector3d())

            //TODO this is dumb, but i forgot to make assembly return center positions, oh well
            val event = VSAssemblyEvents.onPasteBeforeBlocksAreLoaded.on {
                centerPositions = it.centerPosition.first.get(Vector3d()) to it.centerPosition.second.get(Vector3d())
            }
            val shiptraption = assembleToShip(
                level,
                selection.toSet().map { it.toMinecraft() }.toSet(), //accursed, unholy, abominable
                1.0
            )
            event.unregister()

            val newPos = Vector3d(worldPos).sub(centerPositions.first).add(centerPositions.second)

            shiptraptionID = shiptraption.id
            Triple(newPos, shiptraption, direction)
        } else {
            shiptraptionID = otherShip.id
            Triple(otherPos.blockPos.toVector3d() + 0.5 - direction.normal.toJOMLD(), otherShip, otherPos.direction)
        }


        // AllSoundEvents.CONTRAPTION_ASSEMBLE.playOnServer(level, worldPosition);
        ClockworkSounds.PHYSICS_INFUSER_LIGHTNING.playOnServer(level, worldPosition)

        val shipOnID = shipOn?.id

        val posInWorld = shipOn?.transform?.shipToWorld?.transformPosition(
            posInOwnerShip - bearingPos + shiptraption.inertiaData.centerOfMass , Vector3d()
        ) ?: (worldPos - bearingPos + shiptraption.inertiaData.centerOfMass)
        val rotInWorld = shipOn?.transform?.shipToWorldRotation ?: Quaterniond()
        val scaling    = shipOn?.transform?.shipToWorldScaling ?: Vector3d(1.0, 1.0, 1.0)

        shiptraption.unsafeSetTransform(BodyTransformFactory.create(
            posInWorld, rotInWorld, scaling, shiptraption.transform.positionInModel
        ))

        val ship1rot = getHingeRotation(direction)
        val ship2rot = getHingeRotation(direction)

        val extraDist = SERVO_JOINT_ANCHOR_OFFSET
        joint = VSRevoluteJoint(
            shiptraptionID, VSJointPose(bearingPos.fma(-extraDist, axis, Vector3d()), ship1rot),
            shipOnID, VSJointPose(posInOwnerShip.fma(-extraDist, axis, Vector3d()), ship2rot),
            maxForceTorque = computeJointMaxForceTorque(),
            compliance = SERVO_COMPLIANCE,
            driveFreeSpin = true
        )

        this.bearingAxis = axis
        this.bearingPos = bearingPos

        controllerCreationData = PhysBearingData(
            bearingAxis.get(Vector3d()),
            Math.toRadians(targetAngle.toDouble()),
            getRealisticAngularSpeed(),
            shouldUseFixedJoint(movementMode?.get() ?: LockedMode.UNLOCKED, aligning),
            aligning,
            shipOnID ?: -1L,
            joint!!.pose1.pos.get(Vector3d()),
            joint!!.pose0.pos.get(Vector3d())
        )

        resetFixedModeRuntimeState()
        fixedAcquireTicksRemaining = FIXED_ACQUIRE_TICKS
        fixedPostLoadSettleTicksRemaining = FIXED_POST_LOAD_SETTLE_TICKS
        needsJointRematerialization = true
        tryMakeJoint()
        tryUpdateData()

        sendData()
        updateGeneratedRotation()
    }

    override fun destroy() {
        val level = level ?: return
        if (level.isClientSide || level !is ServerLevel) return
        if (reconnectFreezeRequested) {
            releaseReconnectFreeze(level)
        }

        val ship = level.shipObjectWorld.loadedShips.getById(shiptraptionID) ?: return
        BearingController.getOrCreate(ship)!!.removePhysBearing(bearingID)

        joint?.let { level.gtpa.removeJoint(jointID) }
    }

    fun disassemble() {
        if (!isRunning && shiptraptionID == NO_SHIPTRAPTION_ID) return
        if (ticks - lastStateChanged <= cooldown) return
        targetAngle = 0f
        if (shiptraptionID == NO_SHIPTRAPTION_ID) return
        val level = level as ServerLevel
        val ship = level.shipObjectWorld.loadedShips.getById(shiptraptionID) ?: return resetState()

        if (!canDisassemble(bearingAxis, ship, level.getShipObjectManagingPos(worldPosition))) {
            disassembleWhenPossible = !disassembleWhenPossible
            aligning = !aligning
            BearingController.getOrCreate(ship)!!.bearingData[bearingID]?.let { it.aligning = this.aligning }
            fixedModeInitialized = false
            activeFixedTargetRad = null
            fixedDesiredTargetRadContinuous = null
            fixedDesiredTargetInitialized = false
            fixedAcquireTicksRemaining = FIXED_ACQUIRE_TICKS
            fixedPostLoadSettleTicksRemaining = if (!aligning && (movementMode?.get() == LockedMode.FOLLOW_ANGLE)) {
                FIXED_POST_LOAD_SETTLE_TICKS
            } else {
                0
            }
            lastSentFixedTargetRad = null
            lastFixedRefreshTick = -FIXED_TARGET_SAFETY_REFRESH_TICKS
            fixedTrackStepRadPerTick = 0.0
            fixedTrackStepAccelRadPerTick2 = 0.0
            fixedTrackStepInitialized = false
            fixedTrackLastMeasuredAngleRad = null
            fixedWasMovingFollow = false
            fixedStopHoldLatched = false
            fixedStopHoldAngleRad = null
            needsJointRematerialization = true
            tryUpdateData()
        } else {
            shipDisassemble()
        }
        AllSoundEvents.CONTRAPTION_DISASSEMBLE.playOnServer(level, worldPosition)
    }

    private fun shipDisassemble() {
        if (shiptraptionID == NO_SHIPTRAPTION_ID || level!!.isClientSide) { return }
        val level = level as ServerLevel
        val subShip = level.shipObjectWorld.loadedShips.getById(shiptraptionID) ?: return
        val mainShip = level.getShipObjectManagingPos(worldPosition)

        if (!canDisassemble(bearingAxis, subShip, mainShip)) { return }
        val direction = originalDirection ?: blockState.getValue(BearingBlock.FACING)
        val inMain = worldPosition.relative(direction, 1)
        val inSubship = bearingPos.add(bearingAxis, Vector3d()).let { BlockPos.containing(it.x, it.y, it.z) }

        //todo this is stupid
        val aabb = subShip.shipAABB!!
        val blocks = DenseBlockPosSet()
        for (x in aabb.minX() - 1 until  aabb.maxX() + 1) {
        for (z in aabb.minZ() - 1 until  aabb.maxZ() + 1) {
        for (y in aabb.minY() - 1 until  aabb.maxY() + 1) {
            blocks.add(x, y, z)
        } } }

        val subCouldSplit = subShip.getAttachment<SplittingDisablerAttachment>()?.let { if (it.canSplit()) { it.disableSplitting(); true } else {false} } ?: false
        val mainCouldSplit = mainShip?.getAttachment<SplittingDisablerAttachment>()?.let { if (it.canSplit()) { it.disableSplitting(); true } else {false} } ?: false

        val hasMoved = PhysBearingAssembler.moveBlocksFromTo(level, blocks, true, inSubship, inMain, subShip, mainShip)

        if (subCouldSplit) { subShip.getAttachment<SplittingDisablerAttachment>()?.enableSplitting() }
        if (mainCouldSplit) { mainShip?.getAttachment<SplittingDisablerAttachment>()?.enableSplitting() }

        if (!hasMoved) {
            aligning = false
            assembleNextTick = false
            disassembleWhenPossible = false
            return
        }
        BearingController.getOrCreate(subShip)!!.removePhysBearing(bearingID)

        lastStateChanged = ticks
        resetState()
    }

    private fun resetState() {
        if (reconnectFreezeRequested) {
            releaseReconnectFreeze(level as? ServerLevel)
        }
        bearingID = -1
        shiptraptionID = NO_SHIPTRAPTION_ID
        isRunning = false
        updateGeneratedRotation()
        assembleNextTick = false
        disassembleWhenPossible = false
        sequencedAngleLimit = -1.0f
        sequencedAngleProgress = 0f
        targetAngle = 0f
        sendData()
        jointID = -1
        aligning = false
        joint = null
        lastMode = LockedMode.UNLOCKED
        lastSpeed = 0f
        lastAligningState = false
        servoMode = LockedMode.UNLOCKED
        lockedHoldAngleRad = null
        resetFixedModeRuntimeState()
        lastAngle = 0f
        curAngle = 0f
    }

    private fun tryAssembleNextTick() {
        if (!assembleNextTick) {return}
        if (ticks - lastStateChanged <= cooldown) {return}
        assembleNextTick = false
        if (!isRunning) {assemble()}
    }

    private fun tryUpdateData() {
        if (shiptraptionID == NO_SHIPTRAPTION_ID) return
        val level = level as? ServerLevel ?: return
        val mode = movementMode?.get() ?: LockedMode.UNLOCKED
        val speedNow = getSpeed()
        val aligningNow = aligning

        val modeChanged = lastMode != mode
        val speedChanged = lastSpeed != speedNow
        val aligningChanged = lastAligningState != aligningNow
        val modeOrAlignmentTransition = modeChanged || aligningChanged
        val followCommandActive = PhysBearingFollowController.isFollowCommandActive(
            modeName = mode.name,
            aligning = aligningNow,
            commandedSpeed = speedNow,
            speedEps = FOLLOW_COMMAND_ACTIVE_SPEED_EPS
        )

        if (modeChanged && mode == LockedMode.FOLLOW_ANGLE) {
            currentJointAngleRad(level)?.let { angleRad ->
                // FOLLOW enters from live physical angle to avoid startup snap.
                targetAngle = PhysBearingFollowController.normalizeDisplayAngleDeg720(Math.toDegrees(angleRad))
            }
            lockedHoldAngleRad = null
        } else if (modeChanged && mode == LockedMode.LOCKED) {
            lockedHoldAngleRad = Math.toRadians(targetAngle.toDouble())
        } else if (lastMode == LockedMode.LOCKED && mode != LockedMode.LOCKED) {
            lockedHoldAngleRad = null
        }

        val fixedMode = shouldUseFixedJoint(mode, aligningNow)
        val jointKindMismatch = when (val currentJoint = joint) {
            null -> false
            is VSFixedJoint -> !fixedMode
            is VSRevoluteJoint -> fixedMode
            else -> true
        }
        if (jointKindMismatch) needsJointRematerialization = true

        if (fixedMode && modeOrAlignmentTransition) {
            fixedModeInitialized = false
            activeFixedTargetRad = null
            fixedZeroRelRotMainToSub = null
            fixedReferenceCaptured = false
            fixedReferenceAngleRad = 0.0
            fixedAnchorPose0Local = null
            fixedAnchorPose1Local = null
            fixedAnchorCaptured = false
            fixedDesiredTargetRadContinuous = null
            fixedDesiredTargetInitialized = false
            fixedAcquireTicksRemaining = FIXED_ACQUIRE_TICKS
            fixedPostLoadSettleTicksRemaining = if (mode == LockedMode.FOLLOW_ANGLE && !aligningNow) {
                FIXED_POST_LOAD_SETTLE_TICKS
            } else {
                0
            }
            lastSentFixedTargetRad = null
            lastFixedRefreshTick = -FIXED_TARGET_SAFETY_REFRESH_TICKS
            fixedTrackStepRadPerTick = 0.0
            fixedTrackStepAccelRadPerTick2 = 0.0
            fixedTrackStepInitialized = false
            fixedTrackLastMeasuredAngleRad = null
            fixedWasMovingFollow = false
            fixedStopHoldLatched = false
            fixedStopHoldAngleRad = null
        } else if (!fixedMode) {
            activeFixedTargetRad = null
            fixedZeroRelRotMainToSub = null
            fixedReferenceCaptured = false
            fixedReferenceAngleRad = 0.0
            fixedAnchorPose0Local = null
            fixedAnchorPose1Local = null
            fixedAnchorCaptured = false
            fixedDesiredTargetRadContinuous = null
            fixedDesiredTargetInitialized = false
            fixedModeInitialized = false
            fixedAcquireTicksRemaining = 0
            fixedPostLoadSettleTicksRemaining = 0
            lastSentFixedTargetRad = null
            fixedTrackStepRadPerTick = 0.0
            fixedTrackStepAccelRadPerTick2 = 0.0
            fixedTrackStepInitialized = false
            fixedTrackLastMeasuredAngleRad = null
            fixedWasMovingFollow = false
            fixedStopHoldLatched = false
            fixedStopHoldAngleRad = null
        }

        var forcedFixedTargetRad: Double? = null
        var forcedJointAuthority: VSJointMaxForceTorque? = null
        val shouldApplyDrive = if (fixedMode) {
            val desiredWrappedRad = fixedTargetAngleRad(mode, aligningNow)
            val measuredAngleRad = currentJointAngleRad(level)

            if (!fixedModeInitialized || activeFixedTargetRad == null || !fixedReferenceCaptured || fixedZeroRelRotMainToSub == null || !fixedAnchorCaptured || fixedAnchorPose0Local == null || fixedAnchorPose1Local == null) {
                if (
                    measuredAngleRad == null ||
                    !captureFixedZeroReference(level, measuredAngleRad) ||
                    !captureFixedAnchorReference(level)
                ) {
                    needsJointRematerialization = true
                    false
                } else {
                    val inFollowSettleWindow = mode == LockedMode.FOLLOW_ANGLE &&
                        !aligningNow &&
                        fixedPostLoadSettleTicksRemaining > 0
                    val desiredContinuousRad = PhysBearingFollowController.selectDesiredContinuousTarget(
                        wrappedTargetRad = desiredWrappedRad,
                        measuredAngleRad = measuredAngleRad,
                        previousDesiredContinuousRad = null,
                        inFollowSettleWindow = inFollowSettleWindow
                    )
                    fixedDesiredTargetRadContinuous = desiredContinuousRad
                    fixedDesiredTargetInitialized = true
                    activeFixedTargetRad = PhysBearingFollowController.bootstrapFixedTargetAngleRad(
                        measuredAngleRad,
                        desiredContinuousRad
                    )
                    fixedModeInitialized = true
                    fixedAcquireTicksRemaining = FIXED_ACQUIRE_TICKS
                    true
                }
            } else {
                true
            }
        } else {
            false
        }

        val shouldApplyDriveResolved = if (fixedMode) {
            val desiredWrappedRad = fixedTargetAngleRad(mode, aligningNow)
            val measuredAngleRad = currentJointAngleRad(level)

            if (!shouldApplyDrive) {
                false
            } else if (!PhysBearingFollowController.hasValidFixedReferenceContext(
                    measuredAngleRad,
                    fixedReferenceCaptured,
                    fixedZeroRelRotMainToSub,
                    fixedAnchorCaptured,
                    fixedAnchorPose0Local,
                    fixedAnchorPose1Local
                )) {
                needsJointRematerialization = true
                false
            } else {
                val measuredTargetRad = measuredAngleRad!!
                val inFollowSettleWindow = mode == LockedMode.FOLLOW_ANGLE &&
                    !aligningNow &&
                    fixedPostLoadSettleTicksRemaining > 0
                val shouldLatchFollowStop = PhysBearingFollowController.shouldLatchFollowStop(
                    modeName = mode.name,
                    aligning = aligningNow,
                    wasMovingFollow = fixedWasMovingFollow,
                    followCommandActive = followCommandActive
                )
                if (shouldLatchFollowStop) {
                    fixedStopHoldLatched = true
                    fixedStopHoldAngleRad = measuredTargetRad
                    fixedDesiredTargetRadContinuous = measuredTargetRad
                    fixedDesiredTargetInitialized = true
                    activeFixedTargetRad = measuredTargetRad
                    fixedTrackStepRadPerTick = 0.0
                    fixedTrackStepAccelRadPerTick2 = 0.0
                    fixedTrackStepInitialized = true
                    fixedTrackLastMeasuredAngleRad = measuredTargetRad
                    targetAngle = PhysBearingFollowController.normalizeDisplayAngleDeg720(Math.toDegrees(measuredTargetRad))
                    lastAngle = targetAngle
                    curAngle = targetAngle
                } else if (mode == LockedMode.FOLLOW_ANGLE && !aligningNow && followCommandActive) {
                    fixedStopHoldLatched = false
                    fixedStopHoldAngleRad = null
                }

                val latchedStopTargetRad = fixedStopHoldAngleRad
                    ?.takeIf { mode == LockedMode.FOLLOW_ANGLE && !aligningNow && !followCommandActive && fixedStopHoldLatched && it.isFinite() }
                val desiredTargetRad = latchedStopTargetRad ?: PhysBearingFollowController.selectDesiredContinuousTarget(
                    wrappedTargetRad = desiredWrappedRad,
                    measuredAngleRad = measuredTargetRad,
                    previousDesiredContinuousRad = if (fixedDesiredTargetInitialized) fixedDesiredTargetRadContinuous else null,
                    inFollowSettleWindow = inFollowSettleWindow
                )
                fixedDesiredTargetRadContinuous = desiredTargetRad
                fixedDesiredTargetInitialized = true

                val currentActiveTargetRad = activeFixedTargetRad ?: measuredTargetRad
                val commandedStepRadPerTick = Math.toRadians(getActualAngularSpeed().toDouble())
                val measuredStepRadPerTick = fixedTrackLastMeasuredAngleRad?.let { lastMeasured ->
                    shortestAngleErrorRad(measuredTargetRad, lastMeasured)
                }
                val movingFollow = mode == LockedMode.FOLLOW_ANGLE && !aligningNow && followCommandActive && !inFollowSettleWindow

                val steppedTargetRad = when {
                    mode == LockedMode.LOCKED || aligningNow -> {
                        PhysBearingFollowController.stepTowardAngleRad(
                            currentActiveTargetRad,
                            desiredTargetRad,
                            FIXED_LOCK_MAX_STEP_RAD_PER_TICK
                        )
                    }
                    mode == LockedMode.FOLLOW_ANGLE -> {
                        var activeTargetRad = currentActiveTargetRad
                        if (movingFollow) {
                            if (!fixedTrackStepInitialized) {
                                val bootstrapStep = (measuredStepRadPerTick ?: commandedStepRadPerTick)
                                    .takeIf { it.isFinite() } ?: 0.0
                                fixedTrackStepRadPerTick = bootstrapStep.coerceIn(
                                    -FIXED_FOLLOW_TRACK_MAX_STEP_RAD,
                                    FIXED_FOLLOW_TRACK_MAX_STEP_RAD
                                )
                                fixedTrackStepAccelRadPerTick2 = 0.0
                                fixedTrackStepInitialized = true
                            }
                            val jerkState = PhysBearingFollowController.stepJerkLimitedCommand(
                                currentStepRadPerTick = fixedTrackStepRadPerTick,
                                currentAccelRadPerTick2 = fixedTrackStepAccelRadPerTick2,
                                commandedStepRadPerTick = commandedStepRadPerTick,
                                maxAccelRadPerTick2 = FIXED_TRACK_MAX_ACCEL_RAD_PER_TICK2,
                                maxJerkRadPerTick3 = FIXED_TRACK_MAX_JERK_RAD_PER_TICK3,
                                maxAbsStepRadPerTick = FIXED_FOLLOW_TRACK_MAX_STEP_RAD
                            )
                            var filteredStepRad = jerkState.stepRadPerTick
                            if (fixedAcquireTicksRemaining > 0) {
                                filteredStepRad = filteredStepRad.coerceIn(
                                    -FIXED_FOLLOW_ENTRY_MAX_STEP_RAD_PER_TICK,
                                    FIXED_FOLLOW_ENTRY_MAX_STEP_RAD_PER_TICK
                                )
                            }
                            fixedTrackStepRadPerTick = filteredStepRad
                            fixedTrackStepAccelRadPerTick2 = jerkState.accelRadPerTick2
                            activeTargetRad += filteredStepRad

                            val settleProgress = if (inFollowSettleWindow && FIXED_POST_LOAD_SETTLE_TICKS > 0) {
                                1.0 - (
                                    fixedPostLoadSettleTicksRemaining.toDouble() /
                                        FIXED_POST_LOAD_SETTLE_TICKS.toDouble()
                                    )
                            } else {
                                1.0
                            }
                            val catchupGain = if (inFollowSettleWindow && settleProgress < 0.7) {
                                0.0
                            } else {
                                val catchupErrorAbs = abs(shortestAngleErrorRad(desiredTargetRad, activeTargetRad))
                                PhysBearingFollowController.computeAdaptiveCatchupGain(
                                    errorAbsRad = catchupErrorAbs,
                                    minGain = FIXED_TRACK_CATCHUP_MIN_GAIN,
                                    maxGain = FIXED_TRACK_CATCHUP_GAIN,
                                    fullErrorRad = FIXED_TRACK_CATCHUP_FULL_ERROR_RAD
                                )
                            }
                            val catchupError = shortestAngleErrorRad(desiredTargetRad, activeTargetRad)
                            val catchup = catchupError.coerceIn(
                                -FIXED_TRACK_CATCHUP_MAX_RAD_PER_TICK,
                                FIXED_TRACK_CATCHUP_MAX_RAD_PER_TICK
                            )
                            activeTargetRad += catchup * catchupGain
                        } else {
                            if (!inFollowSettleWindow) {
                                fixedTrackStepRadPerTick *= 0.5
                                fixedTrackStepAccelRadPerTick2 *= 0.5
                                if (abs(fixedTrackStepRadPerTick) < FIXED_MOVING_COMMAND_ACTIVE_STEP_RAD) {
                                    fixedTrackStepRadPerTick = 0.0
                                    fixedTrackStepAccelRadPerTick2 = 0.0
                                }
                            }
                            activeTargetRad = PhysBearingFollowController.stepTowardAngleRad(
                                currentActiveTargetRad,
                                desiredTargetRad,
                                FIXED_LOCK_MAX_STEP_RAD_PER_TICK
                            )
                        }
                        activeTargetRad
                    }
                    else -> desiredTargetRad
                }
                activeFixedTargetRad = steppedTargetRad
                forcedFixedTargetRad = steppedTargetRad
                fixedTrackLastMeasuredAngleRad = measuredTargetRad

                val targetDeltaAbsRad = if (lastSentFixedTargetRad == null) {
                    Double.POSITIVE_INFINITY
                } else {
                    abs(shortestAngleErrorRad(steppedTargetRad, lastSentFixedTargetRad!!))
                }
                val driftAbsRad = abs(shortestAngleErrorRad(steppedTargetRad, measuredTargetRad))
                val ticksSinceLastRefresh =
                    if (lastFixedRefreshTick < 0) Int.MAX_VALUE else (ticks - lastFixedRefreshTick)

                val subMass = joint?.shipId0
                    ?.let { level.shipObjectWorld.loadedShips.getById(it) }
                    ?.inertiaData
                    ?.mass
                    ?.toDouble()
                val mainMass = joint?.shipId1
                    ?.let { level.shipObjectWorld.loadedShips.getById(it) }
                    ?.inertiaData
                    ?.mass
                    ?.toDouble()
                val postLoadAuthorityMultiplier = if (inFollowSettleWindow) {
                    PhysBearingFollowController.computePostLoadAuthorityRamp(
                        totalSettleTicks = FIXED_POST_LOAD_SETTLE_TICKS,
                        settleTicksRemaining = fixedPostLoadSettleTicksRemaining,
                        minMultiplier = FIXED_POST_LOAD_AUTHORITY_MIN_MULTIPLIER
                    )
                } else {
                    1.0
                }
                val authorityProfile = PhysBearingFollowController.computeFixedAuthorityProfile(
                    modeName = mode.name,
                    aligning = aligningNow,
                    movingFollow = movingFollow,
                    inPostLoadSettle = inFollowSettleWindow,
                    subMass = subMass,
                    mainMass = mainMass,
                    commandedStepMagnitudeRad = abs(commandedStepRadPerTick),
                    trackingErrorAbsRad = abs(shortestAngleErrorRad(desiredTargetRad, measuredTargetRad)),
                    postLoadAuthorityMultiplier = postLoadAuthorityMultiplier
                )
                forcedJointAuthority = VSJointMaxForceTorque(
                    authorityProfile.maxForce.toFloat(),
                    authorityProfile.maxTorque.toFloat()
                )

                if (fixedAcquireTicksRemaining > 0) fixedAcquireTicksRemaining--
                if (inFollowSettleWindow && fixedPostLoadSettleTicksRemaining > 0) {
                    fixedPostLoadSettleTicksRemaining--
                }

                PhysBearingFollowController.shouldApplyMovingFixedTargetUpdate(
                    movingFollow = movingFollow,
                    inFollowSettleWindow = inFollowSettleWindow,
                    referenceContextValid = true,
                    commandedStepRadPerTick = commandedStepRadPerTick,
                    movingCommandActiveStepRad = FIXED_MOVING_COMMAND_ACTIVE_STEP_RAD,
                    modeOrAlignmentTransition = modeOrAlignmentTransition,
                    jointKindMismatch = needsJointRematerialization || jointKindMismatch || !lastAppliedJointKindFixed,
                    targetDeltaAbsRad = targetDeltaAbsRad,
                    driftAbsRad = driftAbsRad,
                    targetEpsRad = FIXED_TARGET_UPDATE_EPS_RAD,
                    movingTargetEpsRad = authorityProfile.movingTargetEpsRad,
                    holdDriftDeadbandRad = FIXED_HOLD_DRIFT_DEADBAND_RAD,
                    movingDriftForceRad = authorityProfile.movingDriftForceRad,
                    ticksSinceLastRefresh = ticksSinceLastRefresh,
                    safetyRefreshTicks = FIXED_TARGET_SAFETY_REFRESH_TICKS
                )
            }
        } else {
            speedChanged || modeOrAlignmentTransition || needsJointRematerialization || jointKindMismatch || lastAppliedJointKindFixed
        }

        fixedWasMovingFollow = fixedMode && mode == LockedMode.FOLLOW_ANGLE && !aligningNow && followCommandActive
        lastSpeed = speedNow
        lastMode = mode
        lastAligningState = aligningNow

        if (!shouldApplyDriveResolved) return
        if (!updateDrive(forcedFixedTargetRad, forcedJointAuthority)) {
            needsJointRematerialization = true
        }
    }

    private fun tickAnimationLogic() {
        if (inOutCorner < 1 && !cornerShrinking) {
            inOutCorner += 0.0075f
        } else if (inOutCorner >= 1) {
            cornerShrinking = true
        }
        if (inOutCorner > 0 && cornerShrinking) {
            inOutCorner -= 0.0075f
        } else if (inOutCorner <= 0) {
            cornerShrinking = false
        }


        if (isRunning && !open && !opening) {
            opening = true
        }
        if (opening && isRunning && openProgress < 1.0f) {
            openProgress += 0.05f
        } else if (openProgress >= 1.0f) {
            opening = false
            open = true
            openProgress = 1f
        }

        if (open && !isRunning && openProgress > 0.0f) {
            openProgress -= 0.05f
        } else if (openProgress <= 0.0f) {
            open = false
            openProgress = 0.0f
        }
    }

    fun getActualAngularSpeed(): Float {
        val dir = originalDirection!!
        return convertToAngular(getSpeed()) * if (dir == Direction.WEST || dir == Direction.NORTH || dir == Direction.DOWN) 1 else -1
    }

    fun getRealisticAngularSpeed(): Float {
        val dir = originalDirection!!
        return getSpeed() * 2f * PI.toFloat() / 60f * if (dir == Direction.WEST || dir == Direction.NORTH || dir == Direction.DOWN) 1 else -1
    }

    override fun tick() {
        super.tick()
        prevAngle = targetAngle
        ticks++
        if (level!!.isClientSide) clientAngleDiff /= 2f
        if (!level!!.isClientSide) {
            loadingFn?.let { pendingLoad ->
                if (pendingLoad(level as ServerLevel)) {
                    loadingFn = null
                }
            }

            val subShip = (level as ServerLevel).shipObjectWorld.loadedShips.getById(shiptraptionID)
            controllerCreationData?.also {
                bearingID = BearingController
                    .getOrCreate(subShip ?: return@also)!!
                    .addPhysBearing(it)
                controllerCreationData = null
            }
            controllerUpdateData?.also {
                BearingController
                    .getOrCreate(subShip ?: return@also)!!
                    .updatePhysBearing(bearingID, it)
                controllerUpdateData = null
            }
            tryAssembleNextTick()
            if (disassembleWhenPossible) { shipDisassemble() }
        }
        tickAnimationLogic()
        if (!isRunning) return
        val mode = movementMode?.get() ?: LockedMode.UNLOCKED
        if (shiptraptionID == NO_SHIPTRAPTION_ID) {
            targetAngle = 0f
        } else if (joint != null && jointID != -1 && !aligning && mode != LockedMode.LOCKED) {
            val angularSpeed = -getActualAngularSpeed()
            var diff = 0.0f

            if (sequencedAngleLimit >= 0.0f) {
                val sequencedAngleLimit = sequencedAngleLimit * angularSpeed.sign

                sequencedAngleProgress += angularSpeed

                if (angularSpeed > 0 && sequencedAngleProgress > sequencedAngleLimit
                 || angularSpeed < 0 && sequencedAngleProgress < sequencedAngleLimit) {
                    diff = sequencedAngleProgress - sequencedAngleLimit
                    sequencedAngleProgress = sequencedAngleLimit
                }
            }
            val newAngle = targetAngle + angularSpeed - diff
            //this is stupid
            lastAngle = when {
                newAngle >= 360f * 2 -> lastAngle - 360f * 2
                newAngle < 0f -> lastAngle + 360f * 2
                else -> lastAngle
            }
            curAngle = when {
                newAngle >= 360f * 2 -> curAngle - 360f * 2
                newAngle < 0f -> curAngle + 360f * 2
                else -> curAngle
            }
            targetAngle = when {
                newAngle >= 360f * 2 -> newAngle - 360f * 2
                newAngle < 0f -> newAngle + 360f * 2
                else -> newAngle
            }
        }
        //needs to be after targetAngle change
        if (!level!!.isClientSide) { tryUpdateData() }
    }

    override fun onSpeedChanged(previousSpeed: Float) {
        sequencedAngleLimit = -1.0f
        sequencedAngleProgress = 0.0f

        if (sequenceContext != null && sequenceContext.instruction == SequencerInstructions.TURN_ANGLE) {
            sequencedAngleLimit = sequenceContext.getEffectiveValue(theoreticalSpeed.toDouble()).toFloat()
        }

        if (level != null && !level!!.isClientSide && joint != null) {
            lastSpeed = getSpeed()
            tryUpdateData()
        }
        super.onSpeedChanged(previousSpeed)
    }

    override fun lazyTick() {
        super.lazyTick()
        if (shiptraptionID != NO_SHIPTRAPTION_ID && !level!!.isClientSide) sendData()
    }

    override fun addToTooltip(tooltip: List<Component>, isPlayerSneaking: Boolean): Boolean {
        if (super.addToTooltip(tooltip, isPlayerSneaking)) return true
        if (isPlayerSneaking) return false
        if (getSpeed() == 0f) return false
        if (isRunning) return false
        if (blockState.block !is BearingBlock) return false
        val attachedState = level!!.getBlockState(worldPosition.relative(blockState.getValue(BearingBlock.FACING)))
        if (attachedState.canBeReplaced()) return false
        TooltipHelper.addHint(tooltip, "hint.empty_bearing")
        return true
    }

    fun getActualAngle(): Double? {
        val level = level as ServerLevel
        val shiptraption = level.shipObjectWorld.loadedShips.getById(shiptraptionID) ?: return null
        val mainShip = level.getShipManagingPos(worldPosition)
        return getAngle(bearingAxis, shiptraption.transform, mainShip?.transform)
    }

    override fun attach(contraption: ControlledContraptionEntity) {}
    override fun onStall() { if (!level!!.isClientSide) sendData() }
    override fun isValid(): Boolean = !isRemoved
    override fun isAttachedTo(contraption: AbstractContraptionEntity): Boolean = false
    override fun setAngle(forcedAngle: Float) { targetAngle = forcedAngle }
    override fun getLastAssemblyException(): AssemblyException? = lastException
    override fun getBlockPosition(): BlockPos = worldPosition
    override fun isWoodenTop(): Boolean = false

    companion object {
        private data class ReconnectFreezeLease(
            var holders: Int,
            val initialStatic: Boolean
        )

        private val reconnectFreezeLeases: MutableMap<Long, ReconnectFreezeLease> = mutableMapOf()

        @Synchronized
        private fun acquireReconnectFreezeLease(shipId: Long, initialStatic: Boolean) {
            val lease = reconnectFreezeLeases[shipId]
            if (lease == null) {
                reconnectFreezeLeases[shipId] = ReconnectFreezeLease(holders = 1, initialStatic = initialStatic)
            } else {
                lease.holders += 1
            }
        }

        @Synchronized
        private fun releaseReconnectFreezeLease(shipId: Long): Boolean? {
            val lease = reconnectFreezeLeases[shipId] ?: return null
            lease.holders -= 1
            if (lease.holders > 0) return null
            reconnectFreezeLeases.remove(shipId)
            return lease.initialStatic
        }

        const val NO_SHIPTRAPTION_ID: Long = -1

        private const val SERVO_JOINT_FORCE_MAX = 5.0e8
        private const val SERVO_JOINT_TORQUE_MAX = 5.0e8
        private const val SERVO_JOINT_ANCHOR_OFFSET = 1.0
        private const val SERVO_COMPLIANCE = 1.0e-10
        private const val FIXED_ACQUIRE_TICKS = 12
        private const val FIXED_LOCK_MAX_STEP_RAD_PER_TICK = Math.PI / 30.0
        private const val FIXED_FOLLOW_ENTRY_MAX_STEP_RAD_PER_TICK = Math.PI / 15.0
        private const val FIXED_HOLD_DRIFT_DEADBAND_RAD = 0.004363323129985824
        private const val FIXED_TARGET_UPDATE_EPS_RAD = 1.0e-3
        private const val FIXED_TARGET_SAFETY_REFRESH_TICKS = 200
        private const val FIXED_POST_LOAD_SETTLE_TICKS = 12
        private const val FIXED_POST_LOAD_AUTHORITY_MIN_MULTIPLIER = 0.15
        private const val FIXED_MOVING_COMMAND_ACTIVE_STEP_RAD = 5.0e-5
        private const val FOLLOW_COMMAND_ACTIVE_SPEED_EPS = 1.0e-3f
        private const val FIXED_TRACK_MAX_ACCEL_RAD_PER_TICK2 = 0.08
        private const val FIXED_TRACK_MAX_JERK_RAD_PER_TICK3 = 0.20
        private const val FIXED_TRACK_CATCHUP_GAIN = 0.18
        private const val FIXED_TRACK_CATCHUP_MIN_GAIN = 0.04
        private const val FIXED_TRACK_CATCHUP_FULL_ERROR_RAD = 0.3
        private const val FIXED_TRACK_CATCHUP_MAX_RAD_PER_TICK = 0.01
        private const val FIXED_FOLLOW_TRACK_MAX_STEP_RAD = 1.2

        //tolerance is in degrees
        @JvmStatic
        fun canDisassemble(bearingAxis: Vector3d, mainShip: ServerShip, otherShip: ServerShip?, tolerance: Int=5): Boolean {
            if (abs(Math.toDegrees(getAngle(bearingAxis, mainShip.transform, otherShip?.transform))) > tolerance) return false
            return true
        }
    }
}
