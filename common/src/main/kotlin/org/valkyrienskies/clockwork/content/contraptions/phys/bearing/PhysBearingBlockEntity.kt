package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import com.mojang.blaze3d.vertex.PoseStack
import com.simibubi.create.AllSoundEvents
import com.simibubi.create.content.contraptions.AbstractContraptionEntity
import com.simibubi.create.content.contraptions.AssemblyException
import com.simibubi.create.content.contraptions.ControlledContraptionEntity
import com.simibubi.create.content.contraptions.IDisplayAssemblyExceptions
import com.simibubi.create.content.contraptions.bearing.BearingBlock
import com.simibubi.create.content.contraptions.bearing.IBearingBlockEntity
import com.simibubi.create.content.kinetics.base.GeneratingKineticBlockEntity
import com.simibubi.create.content.kinetics.transmission.sequencer.SequencerInstructions
import com.simibubi.create.foundation.blockEntity.behaviour.BehaviourType
import com.simibubi.create.foundation.blockEntity.behaviour.BlockEntityBehaviour
import com.simibubi.create.foundation.blockEntity.behaviour.ValueBoxTransform
import com.simibubi.create.foundation.blockEntity.behaviour.scrollValue.ScrollOptionBehaviour
import com.simibubi.create.foundation.blockEntity.behaviour.scrollValue.ScrollValueBehaviour
import com.simibubi.create.foundation.item.TooltipHelper
import com.simibubi.create.foundation.utility.ServerSpeedProvider
import dev.engine_room.flywheel.lib.transform.TransformStack
import net.createmod.catnip.math.AngleHelper
import net.createmod.catnip.math.VecHelper
import net.minecraft.core.BlockPos
import net.minecraft.core.Direction
import net.minecraft.nbt.CompoundTag
import net.minecraft.network.chat.Component
import net.minecraft.server.level.ServerLevel
import net.minecraft.util.Mth
import net.minecraft.world.level.LevelAccessor
import net.minecraft.world.level.ClipContext
import net.minecraft.world.level.block.entity.BlockEntity
import net.minecraft.world.level.block.entity.BlockEntityType
import net.minecraft.world.level.block.state.BlockState
import net.minecraft.world.phys.Vec3
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
import org.valkyrienskies.mod.common.assembly.ICopyableBlock
import org.valkyrienskies.mod.common.assembly.ShipAssembler.assembleToShip
import org.valkyrienskies.mod.common.assembly.VSAssemblyEvents
import org.valkyrienskies.mod.common.util.SplittingDisablerAttachment
import org.valkyrienskies.mod.common.util.toJOML
import org.valkyrienskies.mod.common.util.toJOMLD
import org.valkyrienskies.mod.common.world.clipIncludeShips
import org.valkyrienskies.mod.util.putVector3d
import java.lang.Math
import kotlin.math.*

class PhysBearingBlockEntity(type: BlockEntityType<*>?, pos: BlockPos?, state: BlockState?) :
    GeneratingKineticBlockEntity(type, pos, state), IBearingBlockEntity, IDisplayAssemblyExceptions,
    ContraptionController, BlockEntityPhysicsListener {

    var assembleNextTick = false
    var movementMode: ScrollOptionBehaviour<LockedMode>? = null
    var isRunning = false
        private set
    var shiptraptionID = NO_SHIPTRAPTION_ID
        private set
    @Volatile var targetAngle = 0f
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
    @Volatile private var aligning = false
    private var bearingAxis: Vector3d = Vector3d()
    private var bearingID: Int = -1

    private var lastSpeed = 0f
    private var lastMode = LockedMode.UNLOCKED
    private var lastAligningState = false
    private var missingSubShipTicks = 0

    private var followAngleStalled = false
    private var followAngleStallTicks = 0
    private var followAngleStallDirSign = 0
    private var lastActualAngleRadForStall: Double? = null
    private var followStallCommandGraceTicks = 0
    private var followStallClearMotionTicks = 0
    private var followStallPrevCmdMagDegPerTick = 0.0
    private var followStallPrevCmdSign = 0

    // Phys-thread reads these; game-thread writes them.
    @Volatile private var servoMode: LockedMode = LockedMode.UNLOCKED
    @Volatile private var lockedHoldAngleRad: Double? = null
    @Volatile private var followOmegaFeedForwardRadSec: Double = 0.0

    private var servoStrengthBehaviour: ServoStrengthScrollValueBehaviour? = null

    @Volatile private var servoStrengthSetting: Int = SERVO_STRENGTH_DEFAULT

    // Derived (phys-thread) servo parameters.
    // Cascaded control:
    //   omega_from_error = clamp(posGain * angle_error)
    //   alpha_cmd = omegaGain * (omega_target - omega_actual)
    @Volatile private var servoPosGain: Double = 0.0
    @Volatile private var servoOmegaGain: Double = 0.0
    @Volatile private var servoPosOmegaLimit: Double = 0.0
    @Volatile private var servoTorqueLimit: Double = 0.0
    @Volatile private var servoSeatWn: Double = 0.0
    @Volatile private var servoSeatDampingRatio: Double = 0.0
    @Volatile private var servoSeatForceLimit: Double = 0.0
    @Volatile private var servoTiltWn: Double = 0.0
    @Volatile private var servoTiltDampingRatio: Double = 0.0
    @Volatile private var servoTiltTorqueLimit: Double = 0.0
    private var physServoTickCounter: Int = 0
    // Phys-thread filtered feedback used to avoid exciting solver jitter near hold.
    @Volatile private var servoOmegaActualFilteredRadSec: Double = 0.0
    @Volatile private var servoAlphaCmdFilteredRadSec2: Double = 0.0
    // Compensates unmodeled downstream inertia (e.g., bearing chains) by matching commanded vs observed response.
    @Volatile private var servoApparentInertiaScale: Double = 1.0
    @Volatile private var servoPrevOmegaActualForApparentInertiaScaleRadSec: Double = Double.NaN
    @Volatile private var servoPrevAppliedHingeTorqueMag: Double = 0.0
    @Volatile private var tickTrackTorqueAuthority: Double = 1.0
    @Volatile private var tickHoldStabilizerAuthority: Double = 1.0
    @Volatile private var tickChainedDynamic: Boolean = false
    @Volatile private var tickGyroRisk: Boolean = false
    @Volatile private var tickFollowModeActive: Boolean = false
    @Volatile private var tickFollowBrakePhase: Boolean = false
    @Volatile private var tickFollowHoldPhase: Boolean = false
    @Volatile private var tickFollowRestStable: Boolean = false
    @Volatile private var tickFollowRigidRest: Boolean = false
    @Volatile private var tickFollowRigidRestBlend: Double = 0.0
    @Volatile private var tickFollowHingeMicroSuppressed: Boolean = false
    @Volatile private var tickFollowSeatMicroSuppressed: Boolean = false
    @Volatile private var tickFollowTiltMicroSuppressed: Boolean = false
    @Volatile private var tickFollowSeatRestBandSatisfied: Boolean = false
    @Volatile private var tickFollowTiltRestBandSatisfied: Boolean = false
    @Volatile private var tickFollowRestUltraStable: Boolean = false
    @Volatile private var omegaEmaRadSec: Double = 0.0
    @Volatile private var appInertiaConsistentTicks: Int = 0
    @Volatile private var followRestStableTicks: Int = 0
    @Volatile private var followRestUltraStableTicks: Int = 0
    @Volatile private var followRigidRestTicks: Int = 0
    @Volatile private var followHingeMicroTicks: Int = 0
    @Volatile private var followSeatMicroTicks: Int = 0
    @Volatile private var followTiltMicroTicks: Int = 0
    @Volatile private var followHoldSettleTicksRemaining: Int = 0
    @Volatile private var followHoldCaptureTicks: Int = 0
    @Volatile private var followHoldEmergencyReleaseTicks: Int = 0
    @Volatile private var followHoldRingDownTicks: Int = 0
    @Volatile private var followHoldLastErrorSign: Int = 0
    @Volatile private var followWasTrackLastTick: Boolean = false
    @Volatile private var followBrakeHoldAngleRad: Double? = null
    @Volatile private var followBrakeRateRadSec: Double = 0.0
    @Volatile private var followBrakeStaticAlphaRadSec2: Double = 0.0
    @Volatile private var followBrakeMaxAlphaRadSec2: Double = 0.0
    @Volatile private var followTrackOmegaGain: Double = 0.0
    @Volatile private var followTrackPosAssistGain: Double = 0.0
    @Volatile private var followTrackPosAssistOmegaLimit: Double = 0.0
    @Volatile private var followTrackAuthorityFloor: Double = AUTH_MIN
    @Volatile private var followTrackMaxOmegaStepPerTick: Double = SERVO_ACTIVE_MAX_OMEGA_STEP_PER_TICK
    @Volatile private var followHoldKpAlphaRadSec2PerRad: Double = 0.0
    @Volatile private var followHoldKdAlphaRadSec: Double = 0.0
    @Volatile private var followHoldDampingZetaMin: Double = 1.0
    @Volatile private var followHoldRingDownKdBoost: Double = 1.0
    @Volatile private var followHoldMaxAlphaRadSec2: Double = 0.0
    @Volatile private var followBrakeSeatKpScale: Double = 1.0
    @Volatile private var followBrakeSeatKdScale: Double = 1.0
    @Volatile private var followBrakeTiltKpScale: Double = 1.0
    @Volatile private var followBrakeTiltKdScale: Double = 1.0
    @Volatile private var followBrakeTiltStiffnessFloor: Double = 0.0
    @Volatile private var followBrakeTiltDeadbandScale: Double = 1.0
    @Volatile private var followChainTorqueScale: Double = 1.0
    @Volatile private var followChainStiffnessScale: Double = 1.0
    @Volatile private var followHoldWorldSeatKpBoost: Double = 1.0
    @Volatile private var followHoldWorldSeatKdBoost: Double = 1.0
    @Volatile private var followHoldWorldTiltKpBoost: Double = 1.0
    @Volatile private var followHoldWorldTiltKdBoost: Double = 1.0
    @Volatile private var followHoldWorldTiltAlphaCapScale: Double = 1.0
    @Volatile private var followHoldWorldTiltAlphaEqCapScale: Double = 1.0
    @Volatile private var followHoldRestTiltStiffnessFloor: Double = FOLLOW_HOLD_TILT_STIFFNESS_FLOOR
    @Volatile private var followHoldSeatBiasKi: Double = 0.0
    @Volatile private var followHoldSeatBiasLeak: Double = 0.0
    @Volatile private var followHoldSeatBiasMaxAccel: Double = 0.0
    @Volatile private var followHoldTiltBiasKi: Double = 0.0
    @Volatile private var followHoldTiltBiasLeak: Double = 0.0
    @Volatile private var followHoldTiltBiasMaxAlpha: Double = 0.0
    @Volatile private var followHoldPostBrakeSettleTicks: Int = 0
    @Volatile private var followHoldPostBrakeKdBoost: Double = 1.0
    @Volatile private var followHoldRestBiasZeroBandScale: Double = 1.0
    @Volatile private var followSeatHoldBiasAccWorld: Vector3d = Vector3d()
    @Volatile private var followTiltHoldBiasAlphaWorld: Vector3d = Vector3d()

    private var controllerCreationData: PhysBearingData? = null
    private var controllerUpdateData: PhysBearingUpdateData? = null
    private var loadingFn: ((ServerLevel) -> Unit)? = null

    init {
        setLazyTickRate(3)
        recomputeServoTuning()
    }

    private fun movementModeChanged(value: Int) {
        if (level == null || level!!.isClientSide) {return}
        // Immediately switch joint behavior when the player changes modes.
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

        servoStrengthBehaviour = ServoStrengthScrollValueBehaviour(
            Component.translatable("$MOD_ID.phys_bearing.servo_strength"),
            this,
            ServoSettingValueBoxTransform(8.0, 4.0)
        ).also {
            it.between(SERVO_STRENGTH_MIN, SERVO_STRENGTH_MAX)
            it.currentValue = servoStrengthSetting
            it.withCallback { v -> setServoStrengthSetting(v, sendUpdate = true) }
            it.requiresWrench()
            behaviours.add(it)
        }
    }

    private fun setServoStrengthSetting(value: Int, sendUpdate: Boolean) {
        val clamped = value.coerceIn(SERVO_STRENGTH_MIN, SERVO_STRENGTH_MAX)
        if (servoStrengthSetting == clamped) return
        servoStrengthSetting = clamped
        recomputeServoTuning()
        if (sendUpdate && level != null && !level!!.isClientSide) {
            updateDrive()
            sendData()
        }
    }

    private fun lerpLog(min: Double, max: Double, t: Double): Double {
        if (min <= 0.0 || max <= 0.0) return min + (max - min) * t
        val lnMin = ln(min)
        val lnMax = ln(max)
        return exp(lnMin + (lnMax - lnMin) * t)
    }

    private fun recomputeServoTuning() {
        val tuneT = servoStrengthSetting.toDouble() / SERVO_STRENGTH_MAX.toDouble()
        val sliderScale = SERVO_SLIDER_TEST_SCALE.coerceAtLeast(0.0)
        val tuneTClamped = tuneT.coerceIn(0.0, 1.0)

        // Single slider drives all control aggressiveness and authority.
        var wn = lerpLog(SERVO_WN_MIN, SERVO_WN_MAX, tuneTClamped) * sliderScale
        val maxError = max(SERVO_MAX_ERROR_RAD, 1.0e-3)
        val wnCap = sqrt(SERVO_MAX_ALPHA_HARD / maxError)
        wn = wn.coerceAtMost(wnCap)
        val dampingRatio = (SERVO_DAMPING_RATIO_MIN +
            (SERVO_DAMPING_RATIO_MAX - SERVO_DAMPING_RATIO_MIN) * tuneTClamped) * sliderScale

        // Cascaded gains:
        // - posGain maps angle error -> target omega.
        // - omegaGain maps omega error -> target angular acceleration.
        servoPosGain = wn
        servoOmegaGain = 2.0 * dampingRatio * wn
        servoPosOmegaLimit = (lerpLog(SERVO_POS_OMEGA_LIMIT_MIN, SERVO_POS_OMEGA_LIMIT_MAX, tuneTClamped) * sliderScale)
            .coerceAtMost(SERVO_MAX_OMEGA)

        // Authority limits.
        servoTorqueLimit =
            lerpLog(SERVO_TORQUE_MIN, SERVO_TORQUE_MAX, tuneTClamped) * sliderScale

        // Anchor-seat controller (position lock).
        val seatWnRaw = lerpLog(SERVO_SEAT_WN_MIN, SERVO_SEAT_WN_MAX, tuneTClamped) * sliderScale
        val seatWnCap = sqrt(SERVO_SEAT_MAX_ACCEL_HARD / max(SERVO_SEAT_MAX_ERROR_M, 1.0e-3))
        servoSeatWn = seatWnRaw.coerceAtMost(seatWnCap)
        servoSeatDampingRatio = (SERVO_SEAT_DAMPING_RATIO_MIN +
            (SERVO_SEAT_DAMPING_RATIO_MAX - SERVO_SEAT_DAMPING_RATIO_MIN) * tuneTClamped) * sliderScale
        servoSeatForceLimit = lerpLog(SERVO_SEAT_FORCE_LIMIT_MIN, SERVO_SEAT_FORCE_LIMIT_MAX, tuneTClamped) * sliderScale

        // Off-axis orientation lock (swing lock, not twist lock).
        val tiltWnRaw = lerpLog(SERVO_TILT_WN_MIN, SERVO_TILT_WN_MAX, tuneTClamped) * sliderScale
        val tiltWnCap = sqrt(SERVO_TILT_MAX_ALPHA_HARD / max(SERVO_TILT_MAX_ERROR_RAD, 1.0e-3))
        servoTiltWn = tiltWnRaw.coerceAtMost(tiltWnCap)
        servoTiltDampingRatio = (SERVO_TILT_DAMPING_RATIO_MIN +
            (SERVO_TILT_DAMPING_RATIO_MAX - SERVO_TILT_DAMPING_RATIO_MIN) * tuneTClamped) * sliderScale
        servoTiltTorqueLimit = lerpLog(SERVO_TILT_TORQUE_MIN, SERVO_TILT_TORQUE_MAX, tuneTClamped) * sliderScale

        val followParams = PhysBearingServoMath.mapFollowStrength(tuneTClamped, sliderScale)
        followBrakeRateRadSec = followParams.brakeRate
        followBrakeStaticAlphaRadSec2 = followParams.brakeStaticAlpha
        followBrakeMaxAlphaRadSec2 = followParams.maxBrakeAlpha
        followTrackOmegaGain = followParams.trackOmegaGain
        followTrackPosAssistGain = followParams.trackPosAssistGain
        followTrackPosAssistOmegaLimit = followParams.trackPosAssistOmegaLimit
        followTrackAuthorityFloor = followParams.trackAuthorityFloor
        followTrackMaxOmegaStepPerTick = followParams.trackMaxOmegaStepPerTick
        followHoldKpAlphaRadSec2PerRad = followParams.holdKpAlpha
        followHoldKdAlphaRadSec = followParams.holdKdAlpha
        followHoldDampingZetaMin = followParams.holdDampingZetaMin
        followHoldRingDownKdBoost = followParams.holdRingDownKdBoost
        followHoldMaxAlphaRadSec2 = followParams.holdMaxAlpha
        followBrakeSeatKpScale = followParams.brakeSeatKpScale
        followBrakeSeatKdScale = followParams.brakeSeatKdScale
        followBrakeTiltKpScale = followParams.brakeTiltKpScale
        followBrakeTiltKdScale = followParams.brakeTiltKdScale
        followBrakeTiltStiffnessFloor = followParams.brakeTiltStiffnessFloor
        followBrakeTiltDeadbandScale = followParams.brakeTiltDeadbandScale
        followChainTorqueScale = followParams.chainTorqueScale
        followChainStiffnessScale = followParams.chainStiffnessScale
        followHoldWorldSeatKpBoost = followParams.holdWorldSeatKpBoost
        followHoldWorldSeatKdBoost = followParams.holdWorldSeatKdBoost
        followHoldWorldTiltKpBoost = followParams.holdWorldTiltKpBoost
        followHoldWorldTiltKdBoost = followParams.holdWorldTiltKdBoost
        followHoldWorldTiltAlphaCapScale = followParams.holdWorldTiltAlphaCapScale
        followHoldWorldTiltAlphaEqCapScale = followParams.holdWorldTiltAlphaEqCapScale
        followHoldRestTiltStiffnessFloor = followParams.holdRestTiltStiffnessFloor
        followHoldSeatBiasKi = followParams.holdSeatBiasKi
        followHoldSeatBiasLeak = followParams.holdSeatBiasLeak
        followHoldSeatBiasMaxAccel = followParams.holdSeatBiasMaxAccel
        followHoldTiltBiasKi = followParams.holdTiltBiasKi
        followHoldTiltBiasLeak = followParams.holdTiltBiasLeak
        followHoldTiltBiasMaxAlpha = followParams.holdTiltBiasMaxAlpha
        followHoldPostBrakeSettleTicks = followParams.holdPostBrakeSettleTicks
        followHoldPostBrakeKdBoost = followParams.holdPostBrakeKdBoost
        followHoldRestBiasZeroBandScale = followParams.holdRestBiasZeroBandScale
    }

    private fun computeJointMaxForceTorque(): VSJointMaxForceTorque {
        val torqueLimit = servoTorqueLimit
        val maxTorque = if (torqueLimit.isFinite() && torqueLimit > 0.0) torqueLimit else SERVO_TORQUE_MIN
        return VSJointMaxForceTorque(SERVO_JOINT_MAX_FORCE, maxTorque.toFloat())
    }

    private fun Vector3dc.isFiniteVec(): Boolean {
        return x().isFinite() && y().isFinite() && z().isFinite()
    }

    private fun maybeReanchorJoint(
        joint: VSRevoluteJoint,
        subShip: PhysShip,
        mainShip: PhysShip?,
        hingeOmegaRadSec: Double
    ): VSRevoluteJoint {
        val chainedDynamic = !subShip.isStatic && mainShip != null && !mainShip.isStatic
        val chainBlend = if (chainedDynamic) chainAuthorityBlend01() else 1.0
        if (SERVO_REANCHOR_PERIOD_TICKS <= 0) return joint
        physServoTickCounter++
        if (physServoTickCounter % SERVO_REANCHOR_PERIOD_TICKS != 0) return joint

        val anchor0World = subShip.transform.shipToWorld.transformPosition(joint.pose0.pos, Vector3d())

        // Anchor 1 is defined by the bearing block position in the main frame (ship or world).
        val pose1Local = Vector3d(worldPosition.center.toJOML()).fma(-SERVO_JOINT_ANCHOR_OFFSET, bearingAxis, Vector3d())
        val anchor1World =
            if (mainShip != null) mainShip.transform.shipToWorld.transformPosition(pose1Local, Vector3d()) else pose1Local

        if (!anchor0World.isFiniteVec() || !anchor1World.isFiniteVec()) return joint
        val subAnchorVel = getPointVelocityWorld(subShip, anchor0World)
        val mainAnchorVel = if (mainShip != null) getPointVelocityWorld(mainShip, anchor1World) else Vector3d()
        val relAnchorVel = mainAnchorVel.sub(subAnchorVel, Vector3d())
        val relAnchorSpeed = relAnchorVel.length()
        val absHingeOmega = abs(hingeOmegaRadSec)
        val maxHingeOmegaForReanchor =
            if (chainedDynamic) {
                lerpClamped(SERVO_REANCHOR_CHAIN_MAX_HINGE_OMEGA_RAD_SEC, SERVO_REANCHOR_MAX_HINGE_OMEGA_RAD_SEC, chainBlend)
            } else {
                SERVO_REANCHOR_MAX_HINGE_OMEGA_RAD_SEC
            }
        val maxRelVelForReanchor =
            if (chainedDynamic) {
                lerpClamped(SERVO_REANCHOR_CHAIN_MAX_REL_VEL_MPS, SERVO_REANCHOR_MAX_REL_VEL_MPS, chainBlend)
            } else {
                SERVO_REANCHOR_MAX_REL_VEL_MPS
            }
        if (!relAnchorSpeed.isFinite() || !absHingeOmega.isFinite()) return joint
        if (absHingeOmega > maxHingeOmegaForReanchor || relAnchorSpeed > maxRelVelForReanchor) return joint

        // As hinge/anchor motion rises, shrink reanchor corrections to avoid injecting impulses.
        val motionRatio = max(
            absHingeOmega / max(maxHingeOmegaForReanchor, 1.0e-6),
            relAnchorSpeed / max(maxRelVelForReanchor, 1.0e-6)
        )
        val motionScale = (1.0 - motionRatio).coerceIn(0.0, 1.0)
        if (motionScale < SERVO_REANCHOR_MIN_MOTION_SCALE) return joint

        val err = anchor0World.distance(anchor1World)
        if (!err.isFinite()) return joint
        if (err < SERVO_REANCHOR_POS_EPS) return joint

        // Don't do large corrections in one go; that can introduce impulses. Nudge toward the desired anchor.
        val deltaWorld = anchor1World.sub(anchor0World, Vector3d())
        val stepScale = if (chainedDynamic) lerpClamped(SERVO_REANCHOR_CHAIN_STEP_SCALE, 1.0, chainBlend) else 1.0
        val baseMaxStep = SERVO_REANCHOR_MAX_STEP * stepScale
        val maxStepNow = baseMaxStep * motionScale
        if (maxStepNow < SERVO_REANCHOR_MIN_STEP) return joint
        val step = min(err, maxStepNow)
        val targetWorld = if (err > 0.0) {
            anchor0World.add(deltaWorld.mul(step / err, Vector3d()), Vector3d())
        } else {
            anchor1World
        }

        val newPose0Pos = subShip.transform.worldToShip.transformPosition(targetWorld, Vector3d())
        val newPose0 = VSJointPose(newPose0Pos, joint.pose0.rot)
        val newPose1 = VSJointPose(pose1Local, joint.pose1.rot)

        if (DEBUG_SERVO) {
            ClockworkMod.LOGGER.info(
                "[PhysBearing] re-anchor (err={}, step={}, motionScale={}, relVel={}, omega={}, jointId={}, ship0={}, ship1={})",
                err,
                step,
                motionScale,
                relAnchorSpeed,
                absHingeOmega,
                jointID,
                joint.shipId0,
                joint.shipId1
            )
        }

        return joint.copy(pose0 = newPose0, pose1 = newPose1)
    }

    private fun updateDrive() {
        val level = level as? ServerLevel ?: return
        val existing = joint ?: return
        val mode = movementMode?.get() ?: LockedMode.UNLOCKED
        val maxForceTorque = computeJointMaxForceTorque()

        // Keep the joint REVOLUTE in all modes; servo is implemented via driveVelocity in physTick.
        val revolute = when (existing) {
            is VSRevoluteJoint -> existing
            is VSFixedJoint -> VSRevoluteJoint(
                existing.shipId0, existing.pose0, existing.shipId1, existing.pose1,
                maxForceTorque = maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                driveFreeSpin = true
            )
            else -> return
        }

        val servoActive = aligning || mode != LockedMode.UNLOCKED
        val baseJoint = if (servoActive) {
            revolute.copy(
                maxForceTorque = maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                // We don't rely on the revolute motor for servo behavior; see physTick torque servo.
                driveVelocity = null,
                driveForceLimit = null,
                driveGearRatio = null,
                driveFreeSpin = false
            )
        } else {
            revolute.copy(
                maxForceTorque = maxForceTorque,
                compliance = SERVO_COMPLIANCE,
                driveVelocity = null,
                driveForceLimit = null,
                driveGearRatio = null,
                driveFreeSpin = true
            )
        }

        joint = baseJoint
        servoMode = mode
        // Sign matches tick() targetAngle integration and BearingController's ideal omega convention.
        followOmegaFeedForwardRadSec =
            if (!aligning && mode == LockedMode.FOLLOW_ANGLE && !followAngleStalled) -getRealisticAngularSpeed().toDouble() else 0.0

        controllerUpdateData = PhysBearingUpdateData(
            Math.toRadians(targetAngle.toDouble()),
            if (mode == LockedMode.UNLOCKED && !aligning) getRealisticAngularSpeed() else 0f,
            servoActive
        )

        if (jointID != -1) {
            level.gtpa.updateJoint(jointID, baseJoint)
        }
    }

    @Volatile override lateinit var dimension: DimensionId
    @Volatile private var lastAngle = targetAngle
    @Volatile private var curAngle = targetAngle

    private fun shortestAngleErrorRad(target: Double, current: Double): Double {
        // Returns error in (-pi, pi].
        val d = target - current
        return atan2(sin(d), cos(d))
    }

    private fun lerpClamped(min: Double, max: Double, t: Double): Double {
        val tc = t.coerceIn(0.0, 1.0)
        return min + (max - min) * tc
    }

    private fun smoothStep(edge0: Double, edge1: Double, x: Double): Double {
        if (edge1 <= edge0) return if (x >= edge1) 1.0 else 0.0
        val t = ((x - edge0) / (edge1 - edge0)).coerceIn(0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)
    }

    private fun lowPass(previous: Double, sample: Double, alpha: Double): Double {
        if (!sample.isFinite()) return previous
        if (!previous.isFinite()) return sample
        val a = alpha.coerceIn(0.0, 1.0)
        return previous + (sample - previous) * a
    }

    private fun clearServoFilterState() {
        servoOmegaActualFilteredRadSec = 0.0
        servoAlphaCmdFilteredRadSec2 = 0.0
        servoApparentInertiaScale = 1.0
        servoPrevOmegaActualForApparentInertiaScaleRadSec = Double.NaN
        servoPrevAppliedHingeTorqueMag = 0.0
        omegaEmaRadSec = 0.0
        appInertiaConsistentTicks = 0
        tickTrackTorqueAuthority = 1.0
        tickHoldStabilizerAuthority = 1.0
        tickChainedDynamic = false
        tickGyroRisk = false
        tickFollowModeActive = false
        tickFollowBrakePhase = false
        tickFollowHoldPhase = false
        tickFollowRestStable = false
        tickFollowRigidRest = false
        tickFollowRigidRestBlend = 0.0
        tickFollowHingeMicroSuppressed = false
        tickFollowSeatMicroSuppressed = false
        tickFollowTiltMicroSuppressed = false
        tickFollowSeatRestBandSatisfied = false
        tickFollowTiltRestBandSatisfied = false
        tickFollowRestUltraStable = false
        followRestStableTicks = 0
        followRestUltraStableTicks = 0
        followRigidRestTicks = 0
        followHingeMicroTicks = 0
        followSeatMicroTicks = 0
        followTiltMicroTicks = 0
        followHoldSettleTicksRemaining = 0
        followHoldCaptureTicks = 0
        followHoldEmergencyReleaseTicks = 0
        followHoldRingDownTicks = 0
        followHoldLastErrorSign = 0
        followWasTrackLastTick = false
        followBrakeHoldAngleRad = null
        followSeatHoldBiasAccWorld = Vector3d()
        followTiltHoldBiasAlphaWorld = Vector3d()
    }

    private fun servoStrength01(): Double {
        return (servoStrengthSetting.toDouble() / SERVO_STRENGTH_MAX.toDouble()).coerceIn(0.0, 1.0)
    }

    private fun chainAuthorityBlend01(): Double {
        // 0 = conservative chain attenuation, 1 = full authority from the strength slider.
        return smoothStep(SERVO_CHAIN_AUTHORITY_BLEND_START, SERVO_CHAIN_AUTHORITY_BLEND_END, servoStrength01())
    }

    private fun apparentInertiaScaleMax(): Double {
        return lerpClamped(
            SERVO_APPARENT_INERTIA_MAX_SCALE_MIN_STRENGTH,
            SERVO_APPARENT_INERTIA_MAX_SCALE_MAX_STRENGTH,
            servoStrength01()
        ).coerceAtLeast(1.0)
    }

    private fun stabilizerApparentInertiaScale(): Double {
        val raw = servoApparentInertiaScale.coerceIn(1.0, apparentInertiaScaleMax())
        val blended = 1.0 + (raw - 1.0) * SERVO_APPARENT_INERTIA_STABILIZER_SHARE
        return blended.coerceIn(1.0, SERVO_APPARENT_INERTIA_STABILIZER_MAX_SCALE)
    }

    private fun updateApparentInertiaScale(
        baseInertia: Double,
        omegaActual: Double,
        errorRad: Double,
        omegaErr: Double
    ): Double {
        val maxScale = apparentInertiaScaleMax()
        val maxScaleLocal = if (tickChainedDynamic) {
            // Harder cap in chains unless slider is very high via apparentInertiaScaleMax().
            lerpClamped(2.0, maxScale, chainAuthorityBlend01())
        } else {
            maxScale
        }
        var scale = servoApparentInertiaScale.coerceIn(1.0, maxScaleLocal)
        val dt = SERVO_PHYS_TICK_DT_SEC
        val prevOmega = servoPrevOmegaActualForApparentInertiaScaleRadSec
        val prevTorque = servoPrevAppliedHingeTorqueMag
        val demandingMotion =
            abs(errorRad) > SERVO_APPARENT_INERTIA_ACTIVE_ERROR_RAD ||
                abs(omegaErr) > SERVO_APPARENT_INERTIA_ACTIVE_OMEGA_ERR_RAD_SEC

        if (demandingMotion) {
            if (
                dt > 1.0e-6 &&
                baseInertia.isFinite() &&
                baseInertia > SERVO_APPARENT_INERTIA_MIN_BASE &&
                prevOmega.isFinite() &&
                prevTorque.isFinite() &&
                abs(prevTorque) > SERVO_APPARENT_INERTIA_MIN_OBS_TORQUE
            ) {
                val alphaObserved = (omegaActual - prevOmega) / dt
                if (alphaObserved.isFinite()) {
                    if (prevTorque * alphaObserved > 0.0) {
                        appInertiaConsistentTicks++
                    } else {
                        appInertiaConsistentTicks = 0
                    }

                    // Debounce: require consistent sign for a few ticks before increasing scale.
                    if (appInertiaConsistentTicks < APP_INERTIA_DEBOUNCE_TICKS) {
                        scale = lowPass(scale, 1.0, SERVO_APPARENT_INERTIA_OPPOSED_DECAY_ALPHA)
                    } else {
                        val alphaDenom = max(abs(alphaObserved), SERVO_APPARENT_INERTIA_ALPHA_FLOOR_RAD_SEC2)
                        val targetScale = (abs(prevTorque) / (baseInertia * alphaDenom)).coerceIn(1.0, maxScaleLocal)

                        val lpAlpha =
                            if (targetScale > scale) SERVO_APPARENT_INERTIA_RISE_ALPHA else SERVO_APPARENT_INERTIA_FALL_ALPHA
                        val proposed = lowPass(scale, targetScale, lpAlpha)

                        // Slew-rate limit to prevent explosive growth from noise.
                        val maxRise = if (tickChainedDynamic) APP_INERTIA_MAX_RISE_PER_TICK_CHAIN else APP_INERTIA_MAX_RISE_PER_TICK
                        val minFall = APP_INERTIA_MIN_FALL_PER_TICK
                        val limited = proposed
                            .coerceAtMost(scale * maxRise)
                            .coerceAtLeast(scale * minFall)

                        scale = limited
                    }
                } else {
                    appInertiaConsistentTicks = 0
                    scale = lowPass(scale, 1.0, SERVO_APPARENT_INERTIA_IDLE_DECAY_ALPHA)
                }
            } else {
                appInertiaConsistentTicks = 0
                scale = lowPass(scale, 1.0, SERVO_APPARENT_INERTIA_IDLE_DECAY_ALPHA)
            }
        } else {
            appInertiaConsistentTicks = 0
            scale = lowPass(scale, 1.0, SERVO_APPARENT_INERTIA_IDLE_DECAY_ALPHA)
        }

        if (
            abs(errorRad) < SERVO_APPARENT_INERTIA_SETTLED_ERROR_RAD &&
            abs(omegaActual) < SERVO_APPARENT_INERTIA_SETTLED_OMEGA_RAD_SEC
        ) {
            scale = lowPass(scale, 1.0, SERVO_APPARENT_INERTIA_SETTLED_DECAY_ALPHA)
        }

        val clamped = if (scale.isFinite()) scale.coerceIn(1.0, maxScaleLocal) else 1.0
        servoApparentInertiaScale = clamped
        servoPrevOmegaActualForApparentInertiaScaleRadSec = omegaActual
        return clamped
    }

    private fun normalizeAngleDeg0To720(angleDeg: Double): Float {
        val period = 360.0 * 2.0
        var wrapped = angleDeg % period
        if (wrapped < 0.0) wrapped += period
        return wrapped.toFloat()
    }

    private fun clearFollowAngleStallState() {
        followAngleStalled = false
        followAngleStallTicks = 0
        followAngleStallDirSign = 0
        lastActualAngleRadForStall = null
        followStallCommandGraceTicks = 0
        followStallClearMotionTicks = 0
        followStallPrevCmdMagDegPerTick = 0.0
        followStallPrevCmdSign = 0
    }

    private fun updateFollowAngleStallState(cmdSpeedDegPerTick: Float) {
        if (level !is ServerLevel) return
        val mode = movementMode?.get() ?: LockedMode.UNLOCKED
        if (!isRunning || aligning || mode != LockedMode.FOLLOW_ANGLE || shiptraptionID == NO_SHIPTRAPTION_ID) {
            if (followAngleStalled || followAngleStallTicks != 0 || lastActualAngleRadForStall != null) {
                clearFollowAngleStallState()
                updateDrive()
                sendData()
            }
            return
        }

        val cmdMag = abs(cmdSpeedDegPerTick).toDouble()
        val cmdSign = cmdSpeedDegPerTick.sign.toInt()
        val cmdOmegaRadSec = if (SERVO_PHYS_TICK_DT_SEC > 1.0e-9) {
            (cmdMag * (Math.PI / 180.0)) / SERVO_PHYS_TICK_DT_SEC
        } else {
            0.0
        }
        val cmdActive = PhysBearingServoMath.isFollowCommandActive(cmdOmegaRadSec, FOLLOW_CMD_EPS_RAD_SEC)
        val cmdStep = PhysBearingServoMath.isSignificantCommandStep(
            previousMagnitude = followStallPrevCmdMagDegPerTick,
            currentMagnitude = cmdMag,
            previousSign = followStallPrevCmdSign,
            currentSign = cmdSign,
            absoluteStepThreshold = FOLLOW_STALL_CMD_STEP_DEG_PER_TICK,
            relativeStepThreshold = FOLLOW_STALL_CMD_STEP_RATIO
        )
        if (cmdStep) {
            followStallCommandGraceTicks = FOLLOW_STALL_CMD_GRACE_TICKS
        } else if (followStallCommandGraceTicks > 0) {
            followStallCommandGraceTicks--
        }
        followStallPrevCmdMagDegPerTick = cmdMag
        followStallPrevCmdSign = cmdSign

        // If the player stops commanding rotation, clear stall so the next input can move again.
        if (!cmdActive || cmdSign == 0) {
            val wasStalled = followAngleStalled
            clearFollowAngleStallState()
            if (wasStalled) {
                if (DEBUG_SERVO) {
                    ClockworkMod.LOGGER.info("[PhysBearing] follow stall cleared (cmd=0)")
                }
                updateDrive()
                sendData()
            }
            return
        }

        val actualRad = getActualAngle() ?: run {
            // Can't observe motion yet; don't accumulate stall ticks.
            followAngleStallTicks = 0
            followStallClearMotionTicks = 0
            lastActualAngleRadForStall = null
            return
        }

        val lastRad = lastActualAngleRadForStall
        lastActualAngleRadForStall = actualRad
        if (lastRad == null) return

        val actualDeltaRad = abs(shortestAngleErrorRad(actualRad, lastRad))
        val actualOmegaAbsRadSec = if (SERVO_PHYS_TICK_DT_SEC > 1.0e-9) {
            actualDeltaRad / SERVO_PHYS_TICK_DT_SEC
        } else {
            0.0
        }

        // If stalled, only clear on command step/reverse or sustained resumed motion.
        if (followAngleStalled) {
            if (cmdSign != followAngleStallDirSign || cmdStep) {
                clearFollowAngleStallState()
                if (DEBUG_SERVO) {
                    ClockworkMod.LOGGER.info("[PhysBearing] follow stall cleared (command change)")
                }
                updateDrive()
                sendData()
                return
            }
            val motionRecovered =
                actualDeltaRad >= FOLLOW_STALL_CLEAR_EPS_RAD ||
                    actualOmegaAbsRadSec >= FOLLOW_STALL_CLEAR_OMEGA_RAD_SEC
            followStallClearMotionTicks = if (motionRecovered) followStallClearMotionTicks + 1 else 0
            if (followStallClearMotionTicks >= FOLLOW_STALL_CLEAR_MOTION_TICKS) {
                clearFollowAngleStallState()
                if (DEBUG_SERVO) {
                    ClockworkMod.LOGGER.info("[PhysBearing] follow stall cleared (motion recovered)")
                }
                updateDrive()
                sendData()
            }
            return
        }

        // Direction changed => restart stall detection.
        if (followAngleStallDirSign != 0 && cmdSign != followAngleStallDirSign) {
            followAngleStallTicks = 0
        }
        followAngleStallDirSign = cmdSign

        val cmdDeltaRad = cmdMag * (Math.PI / 180.0) // deg/tick -> rad/tick
        val epsRad = PhysBearingServoMath.boundedStallEpsilonRad(
            cmdDeltaRad = cmdDeltaRad,
            baseEpsRad = FOLLOW_STALL_EPS_RAD_BASE,
            commandFraction = FOLLOW_STALL_EPS_FRACTION,
            maxEpsRad = FOLLOW_STALL_EPS_RAD_MAX
        )

        val stallCountEligible =
            followStallCommandGraceTicks <= 0 &&
                actualDeltaRad < epsRad &&
                actualOmegaAbsRadSec <= FOLLOW_STALL_MAX_OMEGA_FOR_COUNT_RAD_SEC
        if (stallCountEligible) {
            followAngleStallTicks++
        } else {
            followAngleStallTicks = 0
        }

        if (followAngleStallTicks < FOLLOW_STALL_TICKS) return

        // Stalled: stop advancing the expected angle and remove omega feed-forward so we stop pushing into collisions.
        followAngleStalled = true
        followAngleStallTicks = 0
        followAngleStallDirSign = cmdSign
        followStallClearMotionTicks = 0
        targetAngle = normalizeAngleDeg0To720(Math.toDegrees(actualRad))

        if (DEBUG_SERVO) {
            ClockworkMod.LOGGER.info(
                "[PhysBearing] follow stalled (cmdDegTick={}, actualDeltaRad={}, epsRad={})",
                cmdSpeedDegPerTick,
                actualDeltaRad,
                epsRad
            )
        }

        updateDrive()
        sendData()
    }

    private fun getAngularInertia(physShip: PhysShip, localPos: Vector3dc, axisGlobal: Vector3dc): Double {
        val globalPos: Vector3dc = physShip.transform.shipToWorld.transformPosition(localPos, Vector3d())
        val offset: Vector3dc = globalPos.sub(physShip.transform.positionInWorld, Vector3d())
        return getAngularInertia(
            physShip.momentOfInertia,
            physShip.transform.shipToWorldRotation,
            physShip.mass,
            offset,
            axisGlobal
        )
    }

    private fun getAngularInertia(
        inertiaTensorLocal: Matrix3dc,
        rotation: Quaterniondc,
        mass: Double,
        offsetGlobal: Vector3dc,
        axisGlobal: Vector3dc
    ): Double {
        val offsetPerpToAxis: Vector3dc =
            offsetGlobal.sub(axisGlobal.mul(axisGlobal.dot(offsetGlobal), Vector3d()), Vector3d())
        val axisLocal: Vector3dc = rotation.transformInverse(axisGlobal, Vector3d())
        return inertiaTensorLocal.transform(axisLocal, Vector3d())
            .dot(axisLocal) + offsetPerpToAxis.lengthSquared() * mass
    }

    private fun parallelOperator(left: Double, right: Double): Double {
        return 1.0 / (1.0 / left + 1.0 / right)
    }

    private fun boostedCoupledEquivalent(
        parallelValue: Double,
        firstValue: Double,
        secondValue: Double,
        blend01: Double
    ): Double {
        if (!parallelValue.isFinite() || parallelValue <= 0.0) return parallelValue
        val sumValue = firstValue + secondValue
        if (!sumValue.isFinite() || sumValue <= 0.0) return parallelValue

        // Blend reduced-inertia coupling toward a pseudo-rigid equivalent at high slider.
        val cappedTarget = min(sumValue, parallelValue * SERVO_CHAIN_COUPLED_EQ_MAX_RATIO)
        return lerpClamped(parallelValue, cappedTarget, blend01)
    }

    private fun rejectAlongAxis(vec: Vector3dc, axisUnit: Vector3dc): Vector3d {
        return vec.sub(axisUnit.mul(axisUnit.dot(vec), Vector3d()), Vector3d())
    }

    private fun getPointVelocityWorld(ship: PhysShip, pointWorld: Vector3dc): Vector3d {
        val r = pointWorld.sub(ship.transform.positionInWorld, Vector3d())
        return ship.velocity.add(Vector3d(ship.angularVelocity).cross(r, Vector3d()), Vector3d())
    }

    private fun computeSwingErrorWorld(
        subShip: PhysShip,
        mainShip: PhysShip?,
        axisWorldUnit: Vector3dc
    ): Vector3d {
        val subRotWorld = Quaterniond(subShip.transform.shipToWorldRotation)
        val mainRotWorld = if (mainShip != null) Quaterniond(mainShip.transform.shipToWorldRotation) else Quaterniond()

        // Relative rotation from main to sub.
        val qRel = Quaterniond(mainRotWorld).conjugate().mul(subRotWorld, Quaterniond()).normalize()

        // Decompose qRel into swing/twist around the hinge axis (in main frame), then keep swing only.
        val axisMain = Quaterniond(mainRotWorld).conjugate().transform(axisWorldUnit, Vector3d())
        if (axisMain.lengthSquared() < 1.0e-9) return Vector3d()
        axisMain.normalize()

        val relVecMain = Vector3d(qRel.x, qRel.y, qRel.z)
        val projected = axisMain.mul(relVecMain.dot(axisMain), Vector3d())
        val twist = Quaterniond(projected.x, projected.y, projected.z, qRel.w)
        val twistUnit = if (twist.lengthSquared() < 1.0e-12) Quaterniond() else twist.normalize()
        val swing = qRel.mul(Quaterniond(twistUnit).conjugate(), Quaterniond()).normalize()

        val swingVecMain = Vector3d(swing.x, swing.y, swing.z)
        val swingVecLen = swingVecMain.length()
        if (swingVecLen < 1.0e-9 || !swingVecLen.isFinite()) return Vector3d()

        var angle = 2.0 * atan2(swingVecLen, swing.w.coerceIn(-1.0, 1.0))
        if (angle > Math.PI) angle -= Math.PI * 2.0
        if (!angle.isFinite()) return Vector3d()

        val swingAxisMain = swingVecMain.mul(1.0 / swingVecLen, Vector3d())
        val errorMain = swingAxisMain.mul(angle, Vector3d())
        return mainRotWorld.transform(errorMain, Vector3d())
    }

    private fun applyAnchorSeatStabilizer(
        joint: VSRevoluteJoint,
        subShip: PhysShip,
        mainShip: PhysShip?,
        axisWorldUnit: Vector3dc,
        axisHoldProfile: PhysBearingServoMath.AxisHoldProfile
    ) {
        val subDynamic = !subShip.isStatic
        val mainDynamic = mainShip != null && !mainShip.isStatic
        if (!subDynamic && !mainDynamic) return
        val trackAuthority = tickTrackTorqueAuthority
        val holdAuthority = tickHoldStabilizerAuthority
        val chainedDynamic = tickChainedDynamic
        val gyroRisk = tickGyroRisk
        val followMode = tickFollowModeActive
        val followBrakePhase = tickFollowBrakePhase
        val followHoldPhase = tickFollowHoldPhase
        val followRestStable = tickFollowRestStable
        val followRestUltraStable = tickFollowRestUltraStable
        val followRigidRest = tickFollowRigidRest
        val followRigidRestBlend = tickFollowRigidRestBlend.coerceIn(0.0, 1.0)
        val followHoldSettleActive = followHoldPhase && followHoldSettleTicksRemaining > 0
        val followBrakeOrHold = followBrakePhase || followHoldPhase
        val axisProfile = if (followMode && followHoldPhase) axisHoldProfile else PhysBearingServoMath.AxisHoldProfile.IDENTITY
        val authority = if (followMode && followHoldPhase) holdAuthority else trackAuthority
        if (!followMode || !followHoldPhase) {
            tickFollowSeatMicroSuppressed = false
            tickFollowSeatRestBandSatisfied = false
            followSeatMicroTicks = 0
        }

        val anchor0World = subShip.transform.shipToWorld.transformPosition(joint.pose0.pos, Vector3d())
        val anchor1World = if (mainShip != null) {
            mainShip.transform.shipToWorld.transformPosition(joint.pose1.pos, Vector3d())
        } else {
            joint.pose1.pos.get(Vector3d())
        }
        if (!anchor0World.isFiniteVec() || !anchor1World.isFiniteVec()) return

        // Revolute should keep anchors coincident in all directions.
        val anchorErrorRaw = anchor1World.sub(anchor0World, Vector3d())
        val subAnchorVel = if (subDynamic) getPointVelocityWorld(subShip, anchor0World) else Vector3d()
        val mainAnchorVel = if (mainDynamic) getPointVelocityWorld(mainShip!!, anchor1World) else Vector3d()
        val relVel = mainAnchorVel.sub(subAnchorVel, Vector3d())
        val errorLen = anchorErrorRaw.length()
        val relVelLen = relVel.length()
        val worldAnchored = mainShip == null || mainShip.isStatic
        val subLeverArmToAxis =
            rejectAlongAxis(anchor0World.sub(subShip.transform.positionInWorld, Vector3d()), axisWorldUnit).length().let {
                if (it.isFinite()) it else 0.0
            }
        val mainLeverArmToAxis = if (mainDynamic && mainShip != null) {
            rejectAlongAxis(anchor1World.sub(mainShip.transform.positionInWorld, Vector3d()), axisWorldUnit).length().let {
                if (it.isFinite()) it else 0.0
            }
        } else {
            0.0
        }
        val leverArmToAxis = max(subLeverArmToAxis, mainLeverArmToAxis)
        val heavyHoldBlend =
            if (followMode && followHoldPhase && worldAnchored) {
                PhysBearingServoMath.heavyLeverBlend(
                    leverArmToAxis,
                    FOLLOW_HOLD_HEAVY_LEVER_BLEND_START_M,
                    FOLLOW_HOLD_HEAVY_LEVER_BLEND_END_M
                ) * servoStrength01()
            } else {
                0.0
            }
        val seatBiasMaxAccelNow =
            followHoldSeatBiasMaxAccel *
                lerpClamped(1.0, FOLLOW_HOLD_HEAVY_SEAT_BIAS_CAP_SCALE_MAX, heavyHoldBlend)
        val chainBlend = if (chainedDynamic) chainAuthorityBlend01() else 1.0
        val seatErrorDeadband = if (chainedDynamic) {
            lerpClamped(SERVO_SEAT_CHAIN_ERROR_DEADBAND_M, SERVO_SEAT_ERROR_DEADBAND_M, chainBlend)
        } else {
            SERVO_SEAT_ERROR_DEADBAND_M
        }
        val seatVelDeadband = if (chainedDynamic) {
            lerpClamped(SERVO_SEAT_CHAIN_VEL_DEADBAND, SERVO_SEAT_VEL_DEADBAND, chainBlend)
        } else {
            SERVO_SEAT_VEL_DEADBAND
        }
        val deadbandScale = when {
            gyroRisk -> 3.0
            chainedDynamic -> 2.0
            else -> 1.0
        }
        val seatErrorDeadbandScaled = seatErrorDeadband * deadbandScale
        val seatVelDeadbandScaled = seatVelDeadband * deadbandScale
        val restStableBandScaleBase = lerpClamped(1.0, FOLLOW_REST_STABLE_BAND_SCALE_MIN, servoStrength01())
        val restStableBandScale = restStableBandScaleBase * axisProfile.restGateScale
        val seatMicroState = if (followMode && followHoldPhase) {
            PhysBearingServoMath.stepHysteresisLatch(
                previouslyActive = tickFollowSeatMicroSuppressed,
                previousTicks = followSeatMicroTicks,
                eligible = true,
                enterCondition =
                    errorLen <= FOLLOW_HOLD_MICRO_SEAT_ERROR_ENTER_M &&
                        relVelLen <= FOLLOW_HOLD_MICRO_SEAT_VEL_ENTER_MPS,
                exitCondition =
                    errorLen >= FOLLOW_HOLD_MICRO_SEAT_ERROR_EXIT_M ||
                        relVelLen >= FOLLOW_HOLD_MICRO_SEAT_VEL_EXIT_MPS,
                enterTicksRequired = FOLLOW_HOLD_MICRO_SEAT_ENTER_TICKS
            )
        } else {
            PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        }
        tickFollowSeatMicroSuppressed = seatMicroState.active
        followSeatMicroTicks = seatMicroState.ticks
        val seatRestBandSatisfied =
            errorLen < FOLLOW_REST_STABLE_SEAT_ERROR_BAND_M * restStableBandScale &&
                relVelLen < FOLLOW_REST_STABLE_SEAT_VEL_BAND_MPS * restStableBandScale
        tickFollowSeatRestBandSatisfied = seatRestBandSatisfied
        val seatBiasMag = followSeatHoldBiasAccWorld.length().let { if (it.isFinite()) it else 0.0 }
        val seatBiasActive =
            followMode &&
                followHoldPhase &&
                seatBiasMag > seatBiasMaxAccelNow.coerceAtLeast(1.0e-9) * FOLLOW_HOLD_BIAS_ACTIVE_RATIO
        if (errorLen < seatErrorDeadbandScaled && relVelLen < seatVelDeadbandScaled && !seatBiasActive) return
        if (followMode && followHoldPhase && followRestStable && followRestUltraStable && !followHoldSettleActive) {
            return
        }
        val anchorError = if (errorLen > SERVO_SEAT_MAX_ERROR_M && errorLen > 1.0e-9) {
            anchorErrorRaw.mul(SERVO_SEAT_MAX_ERROR_M / errorLen, Vector3d())
        } else {
            anchorErrorRaw
        }

        val effMassBase = when {
            subDynamic && mainDynamic -> {
                val mainMass = mainShip!!.mass
                val baseParallel = parallelOperator(subShip.mass, mainMass)
                if (chainedDynamic) {
                    boostedCoupledEquivalent(baseParallel, subShip.mass, mainMass, chainBlend)
                } else {
                    baseParallel
                }
            }
            subDynamic -> subShip.mass
            else -> mainShip!!.mass
        }
        if (!effMassBase.isFinite() || effMassBase <= 0.0) return
        val effMass = effMassBase * stabilizerApparentInertiaScale()
        if (!effMass.isFinite() || effMass <= 0.0) return

        val sliderScale = SERVO_SLIDER_TEST_SCALE.coerceAtLeast(0.0)
        val seatWnScale = if (chainedDynamic) lerpClamped(SERVO_SEAT_CHAIN_WN_SCALE, 1.0, chainBlend) else 1.0
        val seatWn = (servoSeatWn.coerceIn(0.0, SERVO_SEAT_WN_MAX * sliderScale) * seatWnScale)
        val seatDamping = servoSeatDampingRatio.coerceIn(0.0, SERVO_SEAT_DAMPING_RATIO_MAX * sliderScale)
        val seatKpScale = when {
            worldAnchored -> SERVO_SEAT_WORLD_ANCHOR_KP_SCALE
            chainedDynamic -> lerpClamped(SERVO_SEAT_CHAIN_KP_SCALE, 1.0, chainBlend)
            else -> 1.0
        }
        val seatKdScale = when {
            worldAnchored -> SERVO_SEAT_WORLD_ANCHOR_KD_SCALE
            chainedDynamic -> lerpClamped(SERVO_SEAT_CHAIN_KD_SCALE, 1.0, chainBlend)
            else -> 1.0
        }
        var seatKp = seatWn * seatWn * seatKpScale
        var seatKd = 2.0 * seatDamping * seatWn * seatKdScale
        if (followMode && chainedDynamic) {
            val minChainStiffness = when {
                followHoldPhase && followRigidRest ->
                    lerpClamped(FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR, FOLLOW_RIGID_REST_CHAIN_STIFFNESS_FLOOR, followRigidRestBlend)
                followHoldPhase -> FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR
                else -> 0.1
            }
            val stiffnessScale = followChainStiffnessScale.coerceIn(minChainStiffness, 1.0)
            seatKp *= stiffnessScale
            seatKd *= (1.0 + (1.0 - stiffnessScale) * FOLLOW_CHAIN_STABILIZER_DAMP_BOOST)
        }
        if (followMode && followBrakeOrHold) {
            seatKp *= followBrakeSeatKpScale
            seatKd *= followBrakeSeatKdScale
            if (followHoldPhase) {
                seatKp *= FOLLOW_HOLD_SEAT_KP_MULT
                seatKd *= FOLLOW_HOLD_SEAT_KD_MULT
                seatKp *= axisProfile.seatKpScale
                seatKd *= axisProfile.seatKdScale
                if (worldAnchored) {
                    seatKp *= followHoldWorldSeatKpBoost
                    seatKd *= followHoldWorldSeatKdBoost
                }
            }
        }
        if (followMode && followHoldPhase && heavyHoldBlend > 0.0) {
            seatKp *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_SEAT_KP_SCALE_MIN, heavyHoldBlend)
            seatKd *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_SEAT_KD_BOOST_MAX, heavyHoldBlend)
        }
        if (
            followMode &&
                followHoldPhase &&
                seatMicroState.active
        ) {
            val rigidBlend = if (followRigidRest) followRigidRestBlend else 0.0
            val microKpScaleBase = lerpClamped(FOLLOW_HOLD_MICRO_SEAT_KP_SCALE, FOLLOW_RIGID_REST_MICRO_SEAT_KP_SCALE, rigidBlend)
            val microKpScale = max(microKpScaleBase, axisProfile.microKpFloor)
            val microKdScaleBase = lerpClamped(FOLLOW_HOLD_MICRO_SEAT_KD_MULT, FOLLOW_RIGID_REST_MICRO_SEAT_KD_MULT, rigidBlend)
            val microKdScale = microKdScaleBase * axisProfile.microKdBoost
            seatKp *= microKpScale
            seatKd *= microKdScale
        }
        if (followMode && followHoldPhase) {
            val settleBlend = smoothStep(
                FOLLOW_HOLD_SETTLE_SEAT_VEL_BLEND_START_MPS,
                FOLLOW_HOLD_SETTLE_SEAT_VEL_BLEND_END_MPS,
                relVelLen
            )
            val verticalSettleBlend = settleBlend * axisProfile.verticalBlend
            seatKp *= lerpClamped(1.0, FOLLOW_HOLD_SETTLE_SEAT_KP_SCALE_MIN, verticalSettleBlend)
            seatKd *= lerpClamped(1.0, FOLLOW_HOLD_SETTLE_SEAT_KD_BOOST_MAX, settleBlend)
        }
        if (followMode && followHoldPhase && followHoldSettleActive) {
            seatKd *= followHoldPostBrakeKdBoost.coerceAtLeast(1.0)
        }
        if (followMode && followHoldPhase) {
            seatKd = PhysBearingServoMath.kdWithFloor(seatKp, seatKd, axisProfile.offAxisDampingZetaMin)
        }

        val velSuppressionBase = 1.0 / (1.0 + relVelLen * 2.0)
        val velSuppression = if (followMode && followHoldPhase) {
            max(FOLLOW_HOLD_SEAT_VEL_SUPPRESSION_FLOOR, velSuppressionBase)
        } else {
            velSuppressionBase
        }
        val seatAuthorityFloor = if (followMode && followHoldPhase) {
            max(FOLLOW_HOLD_AUTH_MIN, holdAuthority)
        } else {
            AUTH_MIN
        }
        val seatAuthority = (authority * velSuppression).coerceIn(seatAuthorityFloor, 1.0)
        val seatAccPd = anchorError.mul(seatKp, Vector3d()).add(relVel.mul(seatKd, Vector3d()))
        val seatBiasAcc =
            if (followMode && followHoldPhase) {
                followSeatHoldBiasAccWorld.get(Vector3d()).takeIf { it.isFiniteVec() } ?: Vector3d()
            } else {
                Vector3d()
            }
        var seatAccCmd = seatAccPd.add(seatBiasAcc)
        seatAccCmd.mul(seatAuthority)
        if (!seatAccCmd.isFiniteVec()) return
        val maxSeatAccel = when {
            worldAnchored -> SERVO_SEAT_MAX_ACCEL_WORLD_ANCHOR
            chainedDynamic -> lerpClamped(SERVO_SEAT_MAX_ACCEL_CHAIN, SERVO_SEAT_MAX_ACCEL_HARD, chainBlend)
            else -> SERVO_SEAT_MAX_ACCEL_HARD
        }
        val accelScale = when {
            gyroRisk -> 0.35
            followMode && followHoldPhase -> 0.85
            chainedDynamic -> 0.60
            else -> 1.0
        }
        var maxSeatAccelScaled = maxSeatAccel * accelScale * seatAuthority
        if (followMode && followHoldPhase && worldAnchored) {
            maxSeatAccelScaled *= axisProfile.worldSeatAccelCapScale
            maxSeatAccelScaled *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_SEAT_ACCEL_CAP_SCALE_MAX, heavyHoldBlend)
        }
        val seatAccLen = seatAccCmd.length()
        var seatAccelSaturated = false
        if (seatAccLen > maxSeatAccelScaled && seatAccLen > 1.0e-9) {
            seatAccCmd.mul(maxSeatAccelScaled / seatAccLen)
            seatAccelSaturated = true
        }

        var seatForce = seatAccCmd.mul(effMass, Vector3d())
        if (!seatForce.isFiniteVec()) return

        val minSeatForce = SERVO_SEAT_FORCE_LIMIT_MIN * sliderScale
        val maxSeatForceCap = SERVO_SEAT_FORCE_LIMIT_MAX * sliderScale
        val maxSeatForce = if (servoSeatForceLimit.isFinite() && servoSeatForceLimit > 0.0) {
            servoSeatForceLimit.coerceIn(minSeatForce, maxSeatForceCap)
        } else {
            minSeatForce
        }
        val forceScale = when {
            gyroRisk -> 0.40
            followMode && followHoldPhase -> 0.90
            chainedDynamic -> 0.70
            else -> 1.0
        }
        val maxSeatForceScaled = maxSeatForce * forceScale * seatAuthority
        val seatForceLen = seatForce.length()
        var seatForceSaturated = false
        if (seatForceLen > maxSeatForceScaled && seatForceLen > 1.0e-9) {
            seatForce.mul(maxSeatForceScaled / seatForceLen)
            seatForceSaturated = true
        }
        if (followMode && followHoldPhase) {
            val seatCorrectiveMotion = anchorError.dot(relVel) < -FOLLOW_HOLD_BIAS_CORRECTIVE_DOT_THRESHOLD
            followSeatHoldBiasAccWorld = PhysBearingServoMath.stepLeakyVectorBias(
                current = followSeatHoldBiasAccWorld,
                error = anchorError,
                dtSec = SERVO_PHYS_TICK_DT_SEC,
                ki = followHoldSeatBiasKi,
                leakPerSec = followHoldSeatBiasLeak,
                maxMagnitude = seatBiasMaxAccelNow,
                freezeIntegrate = seatAccelSaturated || seatForceSaturated || seatCorrectiveMotion
            )
        }

        if (subDynamic) {
            subShip.applyWorldForceToModelPos(seatForce, joint.pose0.pos.get(Vector3d()))
        }
        if (mainDynamic) {
            mainShip!!.applyWorldForceToModelPos(seatForce.mul(-1.0, Vector3d()), joint.pose1.pos.get(Vector3d()))
        }
    }

    private fun applyOffAxisAngularStabilizer(
        joint: VSRevoluteJoint,
        subShip: PhysShip,
        mainShip: PhysShip?,
        axisWorldUnit: Vector3dc,
        relOmegaWorld: Vector3dc,
        axisHoldProfile: PhysBearingServoMath.AxisHoldProfile
    ) {
        val subDynamic = !subShip.isStatic
        val mainDynamic = mainShip != null && !mainShip.isStatic
        if (!subDynamic && !mainDynamic) return
        val trackAuthority = tickTrackTorqueAuthority
        val holdAuthority = tickHoldStabilizerAuthority
        val chainedDynamic = tickChainedDynamic
        val gyroRisk = tickGyroRisk
        val followMode = tickFollowModeActive
        val followBrakePhase = tickFollowBrakePhase
        val followHoldPhase = tickFollowHoldPhase
        val followRestStable = tickFollowRestStable
        val followRestUltraStable = tickFollowRestUltraStable
        val followRigidRest = tickFollowRigidRest
        val followRigidRestBlend = tickFollowRigidRestBlend.coerceIn(0.0, 1.0)
        val followHoldSettleActive = followHoldPhase && followHoldSettleTicksRemaining > 0
        val followBrakeOrHold = followBrakePhase || followHoldPhase
        val axisProfile = if (followMode && followHoldPhase) axisHoldProfile else PhysBearingServoMath.AxisHoldProfile.IDENTITY
        val authority = if (followMode && followHoldPhase) holdAuthority else trackAuthority
        if (!followMode || !followHoldPhase) {
            tickFollowTiltMicroSuppressed = false
            tickFollowTiltRestBandSatisfied = false
            followTiltMicroTicks = 0
        }

        val swingErrorWorldRaw = computeSwingErrorWorld(subShip, mainShip, axisWorldUnit)
        val swingErrorLen = swingErrorWorldRaw.length()
        val swingErrorWorld = if (swingErrorLen > SERVO_TILT_MAX_ERROR_RAD && swingErrorLen > 1.0e-9) {
            swingErrorWorldRaw.mul(SERVO_TILT_MAX_ERROR_RAD / swingErrorLen, Vector3d())
        } else {
            swingErrorWorldRaw
        }
        val offAxisOmega = rejectAlongAxis(relOmegaWorld, axisWorldUnit)
        val offAxisOmegaLen = offAxisOmega.length()
        val worldAnchored = mainShip == null || mainShip.isStatic
        val anchor0World = subShip.transform.shipToWorld.transformPosition(joint.pose0.pos, Vector3d())
        val anchor1World = if (mainShip != null) {
            mainShip.transform.shipToWorld.transformPosition(joint.pose1.pos, Vector3d())
        } else {
            joint.pose1.pos.get(Vector3d())
        }
        val subLeverArmToAxis =
            rejectAlongAxis(anchor0World.sub(subShip.transform.positionInWorld, Vector3d()), axisWorldUnit).length().let {
                if (it.isFinite()) it else 0.0
            }
        val mainLeverArmToAxis = if (mainDynamic && mainShip != null) {
            rejectAlongAxis(anchor1World.sub(mainShip.transform.positionInWorld, Vector3d()), axisWorldUnit).length().let {
                if (it.isFinite()) it else 0.0
            }
        } else {
            0.0
        }
        val leverArmToAxis = max(subLeverArmToAxis, mainLeverArmToAxis)
        val heavyHoldBlend =
            if (followMode && followHoldPhase && worldAnchored) {
                PhysBearingServoMath.heavyLeverBlend(
                    leverArmToAxis,
                    FOLLOW_HOLD_HEAVY_LEVER_BLEND_START_M,
                    FOLLOW_HOLD_HEAVY_LEVER_BLEND_END_M
                ) * servoStrength01()
            } else {
                0.0
            }
        val tiltBiasMaxAlphaNow =
            followHoldTiltBiasMaxAlpha *
                lerpClamped(1.0, FOLLOW_HOLD_HEAVY_TILT_BIAS_CAP_SCALE_MAX, heavyHoldBlend)
        val chainBlend = if (chainedDynamic) chainAuthorityBlend01() else 1.0
        val tiltOmegaDeadband = if (chainedDynamic) {
            lerpClamped(SERVO_TILT_CHAIN_OMEGA_DEADBAND, SERVO_TILT_OMEGA_DEADBAND, chainBlend)
        } else {
            SERVO_TILT_OMEGA_DEADBAND
        }
        val tiltDeadbandScale = when {
            gyroRisk -> 6.0
            chainedDynamic -> 3.0
            else -> 1.0
        }
        val followTiltDeadbandScale = if (followMode && followBrakeOrHold) {
            followBrakeTiltDeadbandScale.coerceAtLeast(0.05)
        } else {
            1.0
        }
        val tiltOmegaDeadbandScaled = tiltOmegaDeadband * tiltDeadbandScale * followTiltDeadbandScale
        val restStableBandScaleBase = lerpClamped(1.0, FOLLOW_REST_STABLE_BAND_SCALE_MIN, servoStrength01())
        val restStableBandScale = restStableBandScaleBase * axisProfile.restGateScale
        val tiltMicroState = if (followMode && followHoldPhase) {
            PhysBearingServoMath.stepHysteresisLatch(
                previouslyActive = tickFollowTiltMicroSuppressed,
                previousTicks = followTiltMicroTicks,
                eligible = true,
                enterCondition =
                    swingErrorWorld.length() <= FOLLOW_HOLD_MICRO_TILT_ERROR_ENTER_RAD &&
                        offAxisOmegaLen <= FOLLOW_HOLD_MICRO_TILT_OMEGA_ENTER_RAD_SEC,
                exitCondition =
                    swingErrorWorld.length() >= FOLLOW_HOLD_MICRO_TILT_ERROR_EXIT_RAD ||
                        offAxisOmegaLen >= FOLLOW_HOLD_MICRO_TILT_OMEGA_EXIT_RAD_SEC,
                enterTicksRequired = FOLLOW_HOLD_MICRO_TILT_ENTER_TICKS
            )
        } else {
            PhysBearingServoMath.HysteresisLatchState(active = false, ticks = 0)
        }
        tickFollowTiltMicroSuppressed = tiltMicroState.active
        followTiltMicroTicks = tiltMicroState.ticks
        val tiltRestBandSatisfied =
            offAxisOmegaLen < FOLLOW_REST_STABLE_TILT_OMEGA_BAND_RAD_SEC * restStableBandScale &&
                swingErrorWorld.length() < FOLLOW_REST_STABLE_TILT_ERROR_BAND_RAD * restStableBandScale
        tickFollowTiltRestBandSatisfied = tiltRestBandSatisfied
        val tiltBiasMag = followTiltHoldBiasAlphaWorld.length().let { if (it.isFinite()) it else 0.0 }
        val tiltBiasActive =
            followMode &&
                followHoldPhase &&
                tiltBiasMag > tiltBiasMaxAlphaNow.coerceAtLeast(1.0e-9) * FOLLOW_HOLD_BIAS_ACTIVE_RATIO
        if ((!offAxisOmegaLen.isFinite() || offAxisOmegaLen < tiltOmegaDeadbandScaled) && swingErrorWorld.length() < 1.0e-6 && !tiltBiasActive) return
        if (followMode && followHoldPhase && followRestStable && followRestUltraStable && !followHoldSettleActive) {
            return
        }

        val sliderScale = SERVO_SLIDER_TEST_SCALE.coerceAtLeast(0.0)
        val tiltWnScale = when {
            worldAnchored -> SERVO_TILT_WORLD_ANCHOR_WN_SCALE
            chainedDynamic -> lerpClamped(SERVO_TILT_CHAIN_WN_SCALE, 1.0, chainBlend)
            else -> 1.0
        }
        val tiltWn = (servoTiltWn.coerceIn(0.0, SERVO_TILT_WN_MAX * sliderScale) * tiltWnScale)
        val tiltDamping = servoTiltDampingRatio.coerceIn(0.0, SERVO_TILT_DAMPING_RATIO_MAX * sliderScale)
        val tiltKpScale = if (chainedDynamic) lerpClamped(SERVO_TILT_CHAIN_KP_SCALE, 1.0, chainBlend) else 1.0
        val tiltKdScale = when {
            worldAnchored -> SERVO_TILT_WORLD_ANCHOR_KD_SCALE
            chainedDynamic -> lerpClamped(SERVO_TILT_CHAIN_KD_SCALE, 1.0, chainBlend)
            else -> 1.0
        }
        var tiltKp = tiltWn * tiltWn * tiltKpScale
        var tiltKd = 2.0 * tiltDamping * tiltWn * tiltKdScale
        if (followMode && chainedDynamic) {
            val minChainStiffness = when {
                followHoldPhase && followRigidRest ->
                    lerpClamped(FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR, FOLLOW_RIGID_REST_CHAIN_STIFFNESS_FLOOR, followRigidRestBlend)
                followHoldPhase -> FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR
                else -> 0.1
            }
            val stiffnessScale = followChainStiffnessScale.coerceIn(minChainStiffness, 1.0)
            tiltKp *= stiffnessScale
            tiltKd *= (1.0 + (1.0 - stiffnessScale) * FOLLOW_CHAIN_STABILIZER_DAMP_BOOST)
        }
        if (followMode && followBrakeOrHold) {
            tiltKp *= followBrakeTiltKpScale
            tiltKd *= followBrakeTiltKdScale
            if (followHoldPhase) {
                tiltKp *= FOLLOW_HOLD_TILT_KP_MULT
                tiltKd *= FOLLOW_HOLD_TILT_KD_MULT
                tiltKp *= axisProfile.tiltKpScale
                tiltKd *= axisProfile.tiltKdScale
                if (worldAnchored) {
                    tiltKp *= followHoldWorldTiltKpBoost
                    tiltKd *= followHoldWorldTiltKdBoost
                }
                val rigidBlend = if (followRigidRest) followRigidRestBlend else 0.0
                tiltKd *= PhysBearingServoMath.holdOffAxisDampingBoost(servoStrength01(), rigidBlend)
            }
        }
        if (followMode && followHoldPhase && heavyHoldBlend > 0.0) {
            tiltKp *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_TILT_KP_SCALE_MIN, heavyHoldBlend)
            tiltKd *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_TILT_KD_BOOST_MAX, heavyHoldBlend)
        }
        if (
            followMode &&
                followHoldPhase &&
                tiltMicroState.active
        ) {
            val rigidBlend = if (followRigidRest) followRigidRestBlend else 0.0
            val microKpScaleBase = lerpClamped(FOLLOW_HOLD_MICRO_TILT_KP_SCALE, FOLLOW_RIGID_REST_MICRO_TILT_KP_SCALE, rigidBlend)
            val microKpScale = max(microKpScaleBase, axisProfile.microKpFloor)
            val microKdScaleBase = lerpClamped(FOLLOW_HOLD_MICRO_TILT_KD_MULT, FOLLOW_RIGID_REST_MICRO_TILT_KD_MULT, rigidBlend)
            val microKdScale = microKdScaleBase * axisProfile.microKdBoost
            tiltKp *= microKpScale
            tiltKd *= microKdScale
        }
        if (followMode && followHoldPhase) {
            val settleBlend = smoothStep(
                FOLLOW_HOLD_SETTLE_TILT_OMEGA_BLEND_START_RAD_SEC,
                FOLLOW_HOLD_SETTLE_TILT_OMEGA_BLEND_END_RAD_SEC,
                offAxisOmegaLen
            )
            val verticalSettleBlend = settleBlend * axisProfile.verticalBlend
            tiltKp *= lerpClamped(1.0, FOLLOW_HOLD_SETTLE_TILT_KP_SCALE_MIN, verticalSettleBlend)
            tiltKd *= lerpClamped(1.0, FOLLOW_HOLD_SETTLE_TILT_KD_BOOST_MAX, settleBlend)
        }
        if (followMode && followHoldPhase && followHoldSettleActive) {
            tiltKd *= followHoldPostBrakeKdBoost.coerceAtLeast(1.0)
        }
        if (followMode && followHoldPhase) {
            tiltKd = PhysBearingServoMath.kdWithFloor(tiltKp, tiltKd, axisProfile.offAxisDampingZetaMin)
        }
        var dampOnlyErrorRad = if (chainedDynamic) {
            lerpClamped(SERVO_TILT_CHAIN_DAMP_ONLY_ERROR_RAD, SERVO_TILT_DAMP_ONLY_ERROR_RAD, chainBlend)
        } else {
            SERVO_TILT_DAMP_ONLY_ERROR_RAD
        }
        var fullStiffnessErrorRad = if (chainedDynamic) {
            lerpClamped(SERVO_TILT_CHAIN_FULL_STIFFNESS_ERROR_RAD, SERVO_TILT_FULL_STIFFNESS_ERROR_RAD, chainBlend)
        } else {
            SERVO_TILT_FULL_STIFFNESS_ERROR_RAD
        }
        if (followMode && followBrakeOrHold) {
            dampOnlyErrorRad *= followTiltDeadbandScale
            fullStiffnessErrorRad *= followTiltDeadbandScale
        }
        val stiffnessAllowed =
            !gyroRisk &&
                offAxisOmegaLen < 0.8 &&
                abs(relOmegaWorld.dot(axisWorldUnit)).let { abs(it) } < 10.0
        val stiffnessBlendBase = if (stiffnessAllowed) {
            smoothStep(dampOnlyErrorRad, fullStiffnessErrorRad, swingErrorWorld.length())
        } else {
            0.0
        }
        var stiffnessBlend = stiffnessBlendBase
        if (followMode && followBrakeOrHold) {
            val floor = followBrakeTiltStiffnessFloor.coerceIn(0.0, 1.0)
            stiffnessBlend = max(stiffnessBlend, floor)
            if (followHoldPhase) {
                val verticalFloorScale =
                    lerpClamped(1.0, FOLLOW_HOLD_SETTLE_VERTICAL_FLOOR_SCALE_MIN, axisProfile.verticalBlend)
                val motionFloorBlend = smoothStep(
                    FOLLOW_HOLD_SETTLE_FLOOR_OMEGA_BLEND_START_RAD_SEC,
                    FOLLOW_HOLD_SETTLE_FLOOR_OMEGA_BLEND_END_RAD_SEC,
                    offAxisOmegaLen
                )
                val motionFloorScale = lerpClamped(1.0, FOLLOW_HOLD_SETTLE_MOTION_FLOOR_SCALE_MIN, motionFloorBlend)
                val effectiveHoldFloor = (
                    followHoldRestTiltStiffnessFloor *
                        verticalFloorScale *
                        motionFloorScale
                    ).coerceIn(FOLLOW_HOLD_SETTLE_MIN_STIFFNESS_FLOOR, followHoldRestTiltStiffnessFloor)
                stiffnessBlend = max(stiffnessBlend, effectiveHoldFloor)
            }
        }

        val tiltAlphaPd = swingErrorWorld.mul(-tiltKp * stiffnessBlend, Vector3d()).add(offAxisOmega.mul(-tiltKd, Vector3d()))
        val tiltBiasAlpha =
            if (followMode && followHoldPhase) {
                followTiltHoldBiasAlphaWorld.get(Vector3d()).takeIf { it.isFiniteVec() } ?: Vector3d()
            } else {
                Vector3d()
            }
        var tiltAlphaCmd = tiltAlphaPd.add(tiltBiasAlpha)
        if (followMode && followHoldPhase) {
            val extraDamping =
                FOLLOW_HOLD_EXTRA_OFFAXIS_DAMPING_RAD_SEC *
                    PhysBearingServoMath.holdOffAxisDampingBoost(
                        servoStrength01(),
                        if (followRigidRest) followRigidRestBlend else 0.0
                    )
            tiltAlphaCmd.add(offAxisOmega.mul(-extraDamping, Vector3d()))
            val axisExtraDamping = axisProfile.extraOffAxisDamping
            if (axisExtraDamping > 0.0) {
                tiltAlphaCmd.add(offAxisOmega.mul(-axisExtraDamping, Vector3d()))
            }
        }
        tiltAlphaCmd.mul(authority)
        if (!tiltAlphaCmd.isFiniteVec()) return
        var tiltAlphaLen = tiltAlphaCmd.length()
        var maxTiltAlpha = when {
            worldAnchored -> SERVO_TILT_MAX_ALPHA_WORLD_ANCHOR
            chainedDynamic -> lerpClamped(SERVO_TILT_MAX_ALPHA_CHAIN, SERVO_TILT_MAX_ALPHA_HARD, chainBlend)
            else -> SERVO_TILT_MAX_ALPHA_HARD
        }
        if (followMode && followBrakeOrHold) {
            maxTiltAlpha *= if (followHoldPhase) FOLLOW_HOLD_TILT_MAX_ALPHA_MULT else FOLLOW_BRAKE_TILT_MAX_ALPHA_MULT
            if (followHoldPhase && worldAnchored) {
                maxTiltAlpha *= followHoldWorldTiltAlphaCapScale
                maxTiltAlpha *= axisProfile.worldTiltAlphaCapScale
                maxTiltAlpha *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_TILT_ALPHA_CAP_SCALE_MAX, heavyHoldBlend)
            }
        }
        var tiltAlphaSaturated = false
        if (tiltAlphaLen > maxTiltAlpha && tiltAlphaLen > 1.0e-9) {
            tiltAlphaCmd.mul(maxTiltAlpha / tiltAlphaLen)
            tiltAlphaLen = tiltAlphaCmd.length()
            tiltAlphaSaturated = true
        }

        val tiltAxis = if (tiltAlphaLen > 1.0e-9) {
            tiltAlphaCmd.mul(1.0 / tiltAlphaLen, Vector3d())
        } else if (offAxisOmegaLen > 1.0e-9) {
            offAxisOmega.mul(1.0 / offAxisOmegaLen, Vector3d())
        } else {
            return
        }

        val effInertiaBase = when {
            subDynamic && mainDynamic -> {
                val subInertia = getAngularInertia(subShip, joint.pose0.pos, tiltAxis)
                val mainInertia = getAngularInertia(mainShip!!, joint.pose1.pos, tiltAxis)
                val baseParallel = parallelOperator(subInertia, mainInertia)
                if (chainedDynamic) {
                    boostedCoupledEquivalent(baseParallel, subInertia, mainInertia, chainBlend)
                } else {
                    baseParallel
                }
            }
            subDynamic -> getAngularInertia(subShip, joint.pose0.pos, tiltAxis)
            else -> getAngularInertia(mainShip!!, joint.pose1.pos, tiltAxis)
        }
        if (!effInertiaBase.isFinite() || effInertiaBase <= 0.0) return
        val effInertia = effInertiaBase * stabilizerApparentInertiaScale()
        if (!effInertia.isFinite() || effInertia <= 0.0) return

        var tiltTorque = tiltAlphaCmd.mul(effInertia, Vector3d())
        if (!tiltTorque.isFiniteVec()) return
        val maxTiltTorque = if (servoTiltTorqueLimit.isFinite() && servoTiltTorqueLimit > 0.0) {
            servoTiltTorqueLimit
        } else {
            SERVO_TILT_TORQUE_MIN * sliderScale
        }
        var maxTiltAlphaEq = when {
            worldAnchored -> SERVO_TILT_MAX_ALPHA_EQUIV_WORLD_ANCHOR
            chainedDynamic -> lerpClamped(SERVO_TILT_MAX_ALPHA_EQUIV_CHAIN, SERVO_TILT_MAX_ALPHA_EQUIV, chainBlend)
            else -> SERVO_TILT_MAX_ALPHA_EQUIV
        }
        if (followMode && followHoldPhase && worldAnchored) {
            maxTiltAlphaEq *= followHoldWorldTiltAlphaEqCapScale
            maxTiltAlphaEq *= axisProfile.worldTiltAlphaEqCapScale
            maxTiltAlphaEq *= lerpClamped(1.0, FOLLOW_HOLD_HEAVY_TILT_ALPHA_EQ_CAP_SCALE_MAX, heavyHoldBlend)
        }
        val maxTiltTorqueByAlpha = effInertia * maxTiltAlphaEq
        val clampedMaxTiltTorque = min(maxTiltTorque, maxTiltTorqueByAlpha)
        val torqueScale = when {
            gyroRisk -> 0.25
            followMode && followHoldPhase && chainedDynamic -> 0.70
            followMode && followHoldPhase -> 0.90
            chainedDynamic -> 0.55
            else -> 1.0
        }
        val clampedMaxTiltTorqueFinal = clampedMaxTiltTorque * torqueScale * authority
        if (!clampedMaxTiltTorqueFinal.isFinite() || clampedMaxTiltTorqueFinal <= 0.0) return
        val tiltTorqueLen = tiltTorque.length()
        var tiltTorqueSaturated = false
        if (tiltTorqueLen > clampedMaxTiltTorqueFinal && tiltTorqueLen > 1.0e-9) {
            tiltTorque.mul(clampedMaxTiltTorqueFinal / tiltTorqueLen)
            tiltTorqueSaturated = true
        }
        if (followMode && followHoldPhase) {
            val tiltBiasError = swingErrorWorld.mul(-1.0, Vector3d())
            val tiltCorrectiveMotion = tiltBiasError.dot(offAxisOmega) < -FOLLOW_HOLD_BIAS_CORRECTIVE_DOT_THRESHOLD
            followTiltHoldBiasAlphaWorld = PhysBearingServoMath.stepLeakyVectorBias(
                current = followTiltHoldBiasAlphaWorld,
                error = tiltBiasError,
                dtSec = SERVO_PHYS_TICK_DT_SEC,
                ki = followHoldTiltBiasKi,
                leakPerSec = followHoldTiltBiasLeak,
                maxMagnitude = tiltBiasMaxAlphaNow,
                freezeIntegrate = tiltAlphaSaturated || tiltTorqueSaturated || tiltCorrectiveMotion
            )
        }

        if (subDynamic) {
            subShip.applyWorldTorque(tiltTorque)
        }
        if (mainDynamic) {
            mainShip!!.applyWorldTorque(tiltTorque.mul(-1.0, Vector3d()))
        }
    }

    override fun physTick(physShip: PhysShip?, physLevel: PhysLevel) {
        if (isRemoved || !isRunning) return
        val jointId = jointID
        if (jointId == -1) return

        val revolute = joint as? VSRevoluteJoint ?: return
        val mode = servoMode
        val servoActive = aligning || mode != LockedMode.UNLOCKED
        val followMode = !aligning && mode == LockedMode.FOLLOW_ANGLE
        if (!servoActive) {
            clearServoFilterState()
            return
        }

        // We need both transforms to compute angle; if ships aren't loaded yet, just wait.
        val subShip = revolute.shipId0?.let { physLevel.getShipById(it) } ?: run {
            if (DEBUG_SERVO) ClockworkMod.LOGGER.info("[PhysBearing] physTick: subShip not loaded yet (jointId={}, ship0={})", jointId, revolute.shipId0)
            return
        }
        val mainShip = revolute.shipId1?.let { physLevel.getShipById(it) }

        val currentAngleRad = getAngle(bearingAxis, subShip.transform, mainShip?.transform)
        val targetAngleRad = when {
            aligning -> 0.0
            mode == LockedMode.LOCKED -> lockedHoldAngleRad ?: currentAngleRad.also { lockedHoldAngleRad = it }
            else -> Math.toRadians(targetAngle.toDouble())
        }
        if (!currentAngleRad.isFinite() || !targetAngleRad.isFinite()) return

        var errorRad = shortestAngleErrorRad(targetAngleRad, currentAngleRad)
        if (abs(errorRad) < SERVO_DEADBAND_RAD) errorRad = 0.0
        val axisWorld = bearingAxis.get(Vector3d())
        mainShip?.transform?.shipToWorldRotation?.transform(axisWorld)
        if (axisWorld.lengthSquared() < 1.0e-9) return
        axisWorld.normalize()

        val relOmegaWorld = Vector3d(subShip.angularVelocity)
        if (mainShip != null) relOmegaWorld.sub(mainShip.angularVelocity)
        val omegaActualRaw = relOmegaWorld.dot(axisWorld) // rad/s along the bearing axis
        if (!omegaActualRaw.isFinite()) return

        // Update omega EMA for stability (reduces solver noise driving estimator/servo).
        omegaEmaRadSec = lowPass(omegaEmaRadSec, omegaActualRaw, OMEGA_EMA_ALPHA)
        val omegaActualForControl = if (omegaEmaRadSec.isFinite()) omegaEmaRadSec else omegaActualRaw

        val offAxisOmegaWorld = rejectAlongAxis(relOmegaWorld, axisWorld)
        val offAxisOmegaLen = offAxisOmegaWorld.length().let { if (it.isFinite()) it else 0.0 }

        // Estimate anchor relative speed cheaply using existing helper (only if mainShip exists):
        val anchor0World = subShip.transform.shipToWorld.transformPosition(revolute.pose0.pos, Vector3d())
        val pose1Local = Vector3d(worldPosition.center.toJOML()).fma(-SERVO_JOINT_ANCHOR_OFFSET, bearingAxis, Vector3d())
        val anchor1World =
            if (mainShip != null) mainShip.transform.shipToWorld.transformPosition(pose1Local, Vector3d()) else pose1Local

        val subAnchorVel = if (anchor0World.isFiniteVec()) getPointVelocityWorld(subShip, anchor0World) else Vector3d()
        val mainAnchorVel = if (mainShip != null && anchor1World.isFiniteVec()) getPointVelocityWorld(mainShip, anchor1World) else Vector3d()
        val relAnchorVel = mainAnchorVel.sub(subAnchorVel, Vector3d())
        val relAnchorSpeed = relAnchorVel.length().let { if (it.isFinite()) it else 0.0 }

        val chainedDynamic = !subShip.isStatic && mainShip != null && !mainShip.isStatic
        tickChainedDynamic = chainedDynamic

        val chainFactor = if (chainedDynamic) AUTH_CHAIN_BASE else 1.0
        val offAxisFactor = 1.0 / (1.0 + offAxisOmegaLen / AUTH_OFFAXIS_SOFT_LIMIT)
        val relVelFactor = 1.0 / (1.0 + relAnchorSpeed / AUTH_RELVEL_SOFT_LIMIT)
        val followTrackAuthorityFloorNow = if (followMode) {
            followTrackAuthorityFloor.coerceIn(AUTH_MIN, 1.0)
        } else {
            AUTH_MIN
        }
        var trackTorqueAuthority = PhysBearingServoMath.computeTrackAuthority(
            chainFactor = chainFactor,
            offAxisFactor = offAxisFactor,
            relVelFactor = relVelFactor,
            minAuthority = AUTH_MIN
        )
        if (followMode) {
            trackTorqueAuthority = max(trackTorqueAuthority, followTrackAuthorityFloorNow)
        }
        var holdStabilizerAuthority = max(
            trackTorqueAuthority,
            PhysBearingServoMath.holdRestAuthorityFloor(servoStrength01(), rigidBlend01 = 0.0)
        ).coerceIn(AUTH_MIN, 1.0)

        val gyroRisk =
            (offAxisOmegaLen > GYRO_RISK_OFFAXIS_OMEGA) ||
                (abs(omegaActualForControl) > GYRO_RISK_HINGE_OMEGA) ||
                (relAnchorSpeed > GYRO_RISK_RELVEL)

        tickGyroRisk = gyroRisk
        if (gyroRisk) {
            val gyroTrackFloor = if (followMode) {
                (followTrackAuthorityFloorNow * FOLLOW_TRACK_AUTH_GYRO_FLOOR_SCALE).coerceIn(AUTH_MIN, 1.0)
            } else {
                AUTH_MIN
            }
            trackTorqueAuthority = max(gyroTrackFloor, trackTorqueAuthority * 0.6)
            holdStabilizerAuthority = max(
                PhysBearingServoMath.holdRestAuthorityFloor(servoStrength01(), rigidBlend01 = 0.0),
                holdStabilizerAuthority * 0.72
            ).coerceIn(AUTH_MIN, 1.0)
        }

        val errorForControl = errorRad.coerceIn(-SERVO_MAX_ERROR_RAD, SERVO_MAX_ERROR_RAD)
        val holdBlend = smoothStep(SERVO_HOLD_BLEND_ERROR_RAD, SERVO_HOLD_RELEASE_ERROR_RAD, abs(errorForControl))

        val omegaFilterAlpha = lerpClamped(SERVO_OMEGA_FILTER_ALPHA_NEAR_HOLD, SERVO_OMEGA_FILTER_ALPHA_ACTIVE, holdBlend)
        servoOmegaActualFilteredRadSec = lowPass(servoOmegaActualFilteredRadSec, omegaActualForControl, omegaFilterAlpha)
        val omegaActual = if (servoOmegaActualFilteredRadSec.isFinite()) {
            servoOmegaActualFilteredRadSec
        } else {
            omegaActualForControl
        }

        var omegaTarget = 0.0
        var omegaErr = 0.0
        var alphaCmdRaw = 0.0
        var omegaFfRaw = 0.0
        var followHoldPhase = false
        var followControlPhase: PhysBearingServoMath.FollowPhase? = null
        var followHoldErrorAbsForRest = Double.POSITIVE_INFINITY
        var followHoldErrorSignedForRest = 0.0
        var followAxisHoldProfile = PhysBearingServoMath.AxisHoldProfile.IDENTITY
        var enteredHoldFromDecel = false

        if (followMode) {
            val cmdOmegaRaw = if (!followAngleStalled) {
                followOmegaFeedForwardRadSec.coerceIn(-SERVO_MAX_OMEGA, SERVO_MAX_OMEGA)
            } else {
                0.0
            }
            val followTrackPhase =
                !followAngleStalled &&
                    PhysBearingServoMath.isFollowCommandActive(abs(cmdOmegaRaw), FOLLOW_CMD_EPS_RAD_SEC)
            val previousPhase = when {
                followWasTrackLastTick -> PhysBearingServoMath.FollowPhase.TRACK
                followBrakeHoldAngleRad != null -> PhysBearingServoMath.FollowPhase.BRAKE_HOLD
                else -> PhysBearingServoMath.FollowPhase.BRAKE_DECEL
            }
            val phaseState = PhysBearingServoMath.stepFollowPhase(
                trackActive = followTrackPhase,
                omegaAbsRadSec = abs(omegaActual),
                previous = PhysBearingServoMath.FollowPhaseState(
                    phase = previousPhase,
                    holdLatched = followBrakeHoldAngleRad != null,
                    captureTicks = followHoldCaptureTicks,
                    emergencyTicks = followHoldEmergencyReleaseTicks
                ),
                captureOmegaRadSec = FOLLOW_BRAKE_HOLD_CAPTURE_OMEGA_RAD_SEC,
                captureTicksRequired = FOLLOW_HOLD_CAPTURE_TICKS,
                emergencyReleaseOmegaRadSec = FOLLOW_HOLD_EMERGENCY_RELEASE_OMEGA_RAD_SEC,
                emergencyReleaseTicksRequired = FOLLOW_HOLD_EMERGENCY_RELEASE_TICKS
            )
            followControlPhase = phaseState.phase
            followHoldPhase = phaseState.phase == PhysBearingServoMath.FollowPhase.BRAKE_HOLD
            followHoldCaptureTicks = phaseState.captureTicks
            followHoldEmergencyReleaseTicks = phaseState.emergencyTicks
            followWasTrackLastTick = phaseState.phase == PhysBearingServoMath.FollowPhase.TRACK
            enteredHoldFromDecel =
                previousPhase == PhysBearingServoMath.FollowPhase.BRAKE_DECEL &&
                    phaseState.phase == PhysBearingServoMath.FollowPhase.BRAKE_HOLD

            when (followControlPhase) {
                PhysBearingServoMath.FollowPhase.TRACK -> {
                    followBrakeHoldAngleRad = null
                    followHoldRingDownTicks = 0
                    followHoldLastErrorSign = 0
                    followHoldSettleTicksRemaining = 0
                    omegaFfRaw = cmdOmegaRaw
                    omegaTarget = PhysBearingServoMath.computeTrackOmegaTarget(
                        commandOmegaRadSec = omegaFfRaw,
                        errorRad = errorForControl,
                        posAssistGain = followTrackPosAssistGain,
                        posAssistOmegaLimit = followTrackPosAssistOmegaLimit,
                        maxOmegaRadSec = SERVO_MAX_OMEGA
                    )
                    omegaErr = omegaTarget - omegaActual
                    alphaCmdRaw = followTrackOmegaGain * omegaErr
                }

                PhysBearingServoMath.FollowPhase.BRAKE_DECEL -> {
                    followBrakeHoldAngleRad = null
                    followHoldRingDownTicks = 0
                    followHoldLastErrorSign = 0
                    followHoldSettleTicksRemaining = 0
                    targetAngle = normalizeAngleDeg0To720(Math.toDegrees(currentAngleRad))
                    val brakeAlphaFriction = PhysBearingServoMath.computeFrictionBrakeAlpha(
                        omegaActual,
                        followBrakeRateRadSec,
                        followBrakeStaticAlphaRadSec2,
                        0.0
                    )
                    val brakeAlphaSnap = PhysBearingServoMath.computeSnapToZeroAlpha(omegaActual, SERVO_PHYS_TICK_DT_SEC)
                    var brakeAlphaRaw = brakeAlphaFriction
                    if (brakeAlphaSnap * omegaActual < 0.0 && abs(brakeAlphaSnap) > abs(brakeAlphaRaw)) {
                        brakeAlphaRaw = brakeAlphaSnap
                    }
                    val maxBrakeAlpha = followBrakeMaxAlphaRadSec2
                        .coerceAtLeast(0.0)
                        .coerceAtMost(SERVO_MAX_ALPHA_HARD)
                    var brakeAlpha = brakeAlphaRaw.coerceIn(-maxBrakeAlpha, maxBrakeAlpha)
                    brakeAlpha = PhysBearingServoMath.clampNoReverse(brakeAlpha, omegaActual, SERVO_PHYS_TICK_DT_SEC)
                    alphaCmdRaw = brakeAlpha
                    omegaTarget = 0.0
                    omegaErr = -omegaActual
                }

                PhysBearingServoMath.FollowPhase.BRAKE_HOLD -> {
                    if (enteredHoldFromDecel) {
                        followHoldSettleTicksRemaining = followHoldPostBrakeSettleTicks.coerceAtLeast(0)
                    }
                    val holdAngle = (followBrakeHoldAngleRad ?: currentAngleRad).let {
                        followBrakeHoldAngleRad = it
                        it
                    }
                    targetAngle = normalizeAngleDeg0To720(Math.toDegrees(holdAngle))
                    val holdError = shortestAngleErrorRad(holdAngle, currentAngleRad)
                        .coerceIn(-SERVO_MAX_ERROR_RAD, SERVO_MAX_ERROR_RAD)
                    followHoldErrorAbsForRest = abs(holdError)
                    followHoldErrorSignedForRest = holdError
                    val ringDownState = PhysBearingServoMath.updateHoldRingDown(
                        currentTicks = followHoldRingDownTicks,
                        previousErrorSign = followHoldLastErrorSign,
                        holdErrorRad = holdError,
                        omegaActualRadSec = omegaActual,
                        errorMinRad = FOLLOW_HOLD_RINGDOWN_ERROR_MIN_RAD,
                        omegaMinRadSec = FOLLOW_HOLD_RINGDOWN_OMEGA_MIN_RAD_SEC,
                        durationTicks = FOLLOW_HOLD_RINGDOWN_TICKS
                    )
                    followHoldRingDownTicks = ringDownState.ticks
                    followHoldLastErrorSign = ringDownState.errorSign
                    val ringDownKdMult = PhysBearingServoMath.holdRingDownKdMultiplier(
                        followHoldRingDownTicks,
                        followHoldRingDownKdBoost
                    )
                    var holdAlpha = PhysBearingServoMath.computeHoldAlpha(
                        holdError,
                        omegaActual,
                        followHoldKpAlphaRadSec2PerRad,
                        followHoldKdAlphaRadSec * ringDownKdMult,
                        followHoldDampingZetaMin,
                        FOLLOW_HOLD_ERROR_DEADBAND_RAD,
                        FOLLOW_HOLD_OMEGA_DEADBAND_RAD_SEC
                    )
                    if (abs(holdError) <= FOLLOW_HOLD_VEL_KILL_ERROR_BAND_RAD) {
                        val snapAlpha = PhysBearingServoMath.computeSnapToZeroAlpha(omegaActual, SERVO_PHYS_TICK_DT_SEC)
                        if (
                            snapAlpha * omegaActual < 0.0 &&
                            abs(omegaActual) <= FOLLOW_HOLD_VEL_KILL_OMEGA_MAX_RAD_SEC &&
                            abs(snapAlpha) > abs(holdAlpha)
                        ) {
                            holdAlpha = snapAlpha
                        }
                    }
                    val maxHoldAlpha = followHoldMaxAlphaRadSec2
                        .coerceAtLeast(0.0)
                        .coerceAtMost(SERVO_MAX_ALPHA_HARD)
                    alphaCmdRaw = holdAlpha.coerceIn(-maxHoldAlpha, maxHoldAlpha)
                    omegaTarget = 0.0
                    omegaErr = -omegaActual
                }
                null -> Unit
            }
        } else {
            followWasTrackLastTick = false
            followBrakeHoldAngleRad = null
            followHoldSettleTicksRemaining = 0
            followHoldCaptureTicks = 0
            followHoldEmergencyReleaseTicks = 0
            followHoldRingDownTicks = 0
            followHoldLastErrorSign = 0
            followRestStableTicks = 0
            followRestUltraStableTicks = 0
            followHoldErrorSignedForRest = 0.0
            tickFollowRestStable = false
            tickFollowRestUltraStable = false
            followRigidRestTicks = 0
            tickFollowRigidRest = false
            followHingeMicroTicks = 0
            tickFollowHingeMicroSuppressed = false
            followSeatMicroTicks = 0
            followTiltMicroTicks = 0
            followSeatHoldBiasAccWorld = Vector3d()
            followTiltHoldBiasAlphaWorld = Vector3d()
            tickFollowSeatMicroSuppressed = false
            tickFollowTiltMicroSuppressed = false
            tickFollowSeatRestBandSatisfied = false
            tickFollowTiltRestBandSatisfied = false
            // Close to hold, fade positional stiffness and let damping dominate.
            val posGainNow = servoPosGain * trackTorqueAuthority
            val omegaGainNow = servoOmegaGain * trackTorqueAuthority
            val omegaFromPositionRaw = (posGainNow * errorForControl).coerceIn(-servoPosOmegaLimit, servoPosOmegaLimit)
            val omegaFromPosition = omegaFromPositionRaw * holdBlend
            omegaTarget = omegaFromPosition.coerceIn(-SERVO_MAX_OMEGA, SERVO_MAX_OMEGA)
            omegaErr = omegaTarget - omegaActual
            alphaCmdRaw = omegaGainNow * omegaErr

            // Prevent tiny setpoint chatter from exciting heavy ships near lock.
            if (
                abs(errorForControl) < SERVO_HOLD_ERROR_RAD &&
                abs(omegaActual) < SERVO_HOLD_OMEGA_RAD_SEC &&
                abs(omegaTarget) < SERVO_HOLD_OMEGA_RAD_SEC
            ) {
                alphaCmdRaw = 0.0
            }
        }

        var followRestStableNow = false
        var rigidRestBlend = 0.0
        if (followMode && followHoldPhase) {
            followAxisHoldProfile = PhysBearingServoMath.computeAxisHoldProfile(abs(axisWorld.y()), servoStrength01())
            val restStableScale =
                lerpClamped(1.0, FOLLOW_REST_STABLE_BAND_SCALE_MIN, servoStrength01()) * followAxisHoldProfile.restGateScale
            val absOmegaHinge = abs(omegaActual)
            val enterRestStable =
                followHoldErrorAbsForRest <= FOLLOW_REST_STABLE_HOLD_ERROR_ENTER_RAD * restStableScale &&
                    absOmegaHinge <= FOLLOW_REST_STABLE_HINGE_OMEGA_ENTER_RAD_SEC * restStableScale &&
                    offAxisOmegaLen <= FOLLOW_REST_STABLE_OFFAXIS_OMEGA_ENTER_RAD_SEC * restStableScale
            val exitRestStable =
                followHoldErrorAbsForRest >= FOLLOW_REST_STABLE_HOLD_ERROR_EXIT_RAD * restStableScale ||
                    absOmegaHinge >= FOLLOW_REST_STABLE_HINGE_OMEGA_EXIT_RAD_SEC * restStableScale ||
                    offAxisOmegaLen >= FOLLOW_REST_STABLE_OFFAXIS_OMEGA_EXIT_RAD_SEC * restStableScale
            if (tickFollowRestStable) {
                followRestStableNow = !exitRestStable
                if (followRestStableNow) {
                    followRestStableTicks = FOLLOW_REST_STABLE_ENTER_TICKS
                } else {
                    followRestStableTicks = 0
                }
            } else {
                followRestStableTicks = if (enterRestStable) followRestStableTicks + 1 else 0
                followRestStableNow = followRestStableTicks >= FOLLOW_REST_STABLE_ENTER_TICKS
            }
        } else {
            followRestStableTicks = 0
            followAxisHoldProfile = PhysBearingServoMath.AxisHoldProfile.IDENTITY
            followRestUltraStableTicks = 0
            tickFollowRestUltraStable = false
            followSeatHoldBiasAccWorld = Vector3d()
            followTiltHoldBiasAlphaWorld = Vector3d()
        }
        tickFollowRestStable = followRestStableNow
        if (followMode && followHoldPhase) {
            val biasZeroBandScale = followHoldRestBiasZeroBandScale.coerceAtLeast(0.01)
            val ultraStableState = PhysBearingServoMath.allowRestEarlyReturn(
                previouslyAllowed = tickFollowRestUltraStable,
                previousTicks = followRestUltraStableTicks,
                seatRestBand = tickFollowSeatRestBandSatisfied,
                tiltRestBand = tickFollowTiltRestBandSatisfied,
                seatMicroBand = tickFollowSeatMicroSuppressed,
                tiltMicroBand = tickFollowTiltMicroSuppressed,
                seatBiasMagnitude = followSeatHoldBiasAccWorld.length().let { if (it.isFinite()) it else Double.POSITIVE_INFINITY },
                tiltBiasMagnitude = followTiltHoldBiasAlphaWorld.length().let { if (it.isFinite()) it else Double.POSITIVE_INFINITY },
                seatBiasZeroBand = FOLLOW_REST_BIAS_ZERO_BAND_SEAT * biasZeroBandScale,
                tiltBiasZeroBand = FOLLOW_REST_BIAS_ZERO_BAND_TILT * biasZeroBandScale,
                enterTicksRequired = FOLLOW_REST_ULTRA_STABLE_ENTER_TICKS
            )
            tickFollowRestUltraStable = ultraStableState.active
            followRestUltraStableTicks = ultraStableState.ticks
        } else {
            tickFollowRestUltraStable = false
            followRestUltraStableTicks = 0
        }
        var followRigidRestNow = false
        rigidRestBlend = smoothStep(FOLLOW_RIGID_REST_BLEND_START, FOLLOW_RIGID_REST_BLEND_END, servoStrength01())
        val rigidRestEligible =
            followMode &&
                followHoldPhase &&
                rigidRestBlend > 0.0
        if (rigidRestEligible) {
            val absOmegaHinge = abs(omegaActual)
            val rigidEnterError = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_ERROR_ENTER_SOFT_RAD,
                FOLLOW_RIGID_REST_HINGE_ERROR_ENTER_HARD_RAD,
                rigidRestBlend
            )
            val rigidEnterOmega = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_OMEGA_ENTER_SOFT_RAD_SEC,
                FOLLOW_RIGID_REST_HINGE_OMEGA_ENTER_HARD_RAD_SEC,
                rigidRestBlend
            )
            val rigidEnterOffAxisOmega = lerpClamped(
                FOLLOW_RIGID_REST_OFFAXIS_OMEGA_ENTER_SOFT_RAD_SEC,
                FOLLOW_RIGID_REST_OFFAXIS_OMEGA_ENTER_HARD_RAD_SEC,
                rigidRestBlend
            )
            val rigidEnterRelVel = lerpClamped(
                FOLLOW_RIGID_REST_RELVEL_ENTER_SOFT_MPS,
                FOLLOW_RIGID_REST_RELVEL_ENTER_HARD_MPS,
                rigidRestBlend
            )
            val rigidExitError = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_ERROR_EXIT_SOFT_RAD,
                FOLLOW_RIGID_REST_HINGE_ERROR_EXIT_HARD_RAD,
                rigidRestBlend
            )
            val rigidExitOmega = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_OMEGA_EXIT_SOFT_RAD_SEC,
                FOLLOW_RIGID_REST_HINGE_OMEGA_EXIT_HARD_RAD_SEC,
                rigidRestBlend
            )
            val rigidExitOffAxisOmega = lerpClamped(
                FOLLOW_RIGID_REST_OFFAXIS_OMEGA_EXIT_SOFT_RAD_SEC,
                FOLLOW_RIGID_REST_OFFAXIS_OMEGA_EXIT_HARD_RAD_SEC,
                rigidRestBlend
            )
            val rigidExitRelVel = lerpClamped(
                FOLLOW_RIGID_REST_RELVEL_EXIT_SOFT_MPS,
                FOLLOW_RIGID_REST_RELVEL_EXIT_HARD_MPS,
                rigidRestBlend
            )
            val rigidEnterTicks = max(
                1,
                lerpClamped(
                    FOLLOW_RIGID_REST_ENTER_TICKS_SOFT.toDouble(),
                    FOLLOW_RIGID_REST_ENTER_TICKS_HARD.toDouble(),
                    rigidRestBlend
                ).roundToInt()
            )
            val rigidEnter =
                followRestStableNow &&
                    followHoldErrorAbsForRest <= rigidEnterError &&
                    absOmegaHinge <= rigidEnterOmega &&
                    offAxisOmegaLen <= rigidEnterOffAxisOmega &&
                    relAnchorSpeed <= rigidEnterRelVel
            val rigidExit =
                followHoldErrorAbsForRest >= rigidExitError ||
                    absOmegaHinge >= rigidExitOmega ||
                    offAxisOmegaLen >= rigidExitOffAxisOmega ||
                    relAnchorSpeed >= rigidExitRelVel
            val rigidState = PhysBearingServoMath.stepHysteresisLatch(
                previouslyActive = tickFollowRigidRest,
                previousTicks = followRigidRestTicks,
                eligible = true,
                enterCondition = rigidEnter,
                exitCondition = rigidExit,
                enterTicksRequired = rigidEnterTicks
            )
            followRigidRestNow = rigidState.active
            followRigidRestTicks = rigidState.ticks
        } else {
            followRigidRestTicks = 0
            rigidRestBlend = 0.0
            followHingeMicroTicks = 0
            tickFollowHingeMicroSuppressed = false
        }
        tickFollowRigidRest = followRigidRestNow
        tickFollowRigidRestBlend = if (followRigidRestNow) rigidRestBlend else 0.0

        holdStabilizerAuthority = max(
            holdStabilizerAuthority,
            PhysBearingServoMath.holdRestAuthorityFloor(
                servoStrength01(),
                rigidBlend01 = if (followRigidRestNow) rigidRestBlend else 0.0
            )
        ).coerceIn(AUTH_MIN, 1.0)

        if (followMode && followHoldPhase && followRigidRestNow) {
            val rigidMicroEnterError = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_ERROR_BAND_SOFT_RAD,
                FOLLOW_RIGID_REST_HINGE_ERROR_BAND_HARD_RAD,
                rigidRestBlend
            )
            val rigidMicroEnterOmega = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_SOFT_RAD_SEC,
                FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_HARD_RAD_SEC,
                rigidRestBlend
            )
            val rigidMicroExitError = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_ERROR_BAND_EXIT_SOFT_RAD,
                FOLLOW_RIGID_REST_HINGE_ERROR_BAND_EXIT_HARD_RAD,
                rigidRestBlend
            )
            val rigidMicroExitOmega = lerpClamped(
                FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_EXIT_SOFT_RAD_SEC,
                FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_EXIT_HARD_RAD_SEC,
                rigidRestBlend
            )
            val rigidMicroEnterTicks = max(
                1,
                lerpClamped(
                    FOLLOW_RIGID_REST_HINGE_MICRO_ENTER_TICKS_SOFT.toDouble(),
                    FOLLOW_RIGID_REST_HINGE_MICRO_ENTER_TICKS_HARD.toDouble(),
                    rigidRestBlend
                ).roundToInt()
            )
            val hingeMicroState = PhysBearingServoMath.stepHysteresisLatch(
                previouslyActive = tickFollowHingeMicroSuppressed,
                previousTicks = followHingeMicroTicks,
                eligible = true,
                enterCondition =
                    followHoldErrorAbsForRest <= rigidMicroEnterError &&
                        abs(omegaActual) <= rigidMicroEnterOmega,
                exitCondition =
                    followHoldErrorAbsForRest >= rigidMicroExitError ||
                        abs(omegaActual) >= rigidMicroExitOmega,
                enterTicksRequired = rigidMicroEnterTicks
            )
            tickFollowHingeMicroSuppressed = hingeMicroState.active
            followHingeMicroTicks = hingeMicroState.ticks
            if (hingeMicroState.active) {
                followHoldRingDownTicks = 0
                followHoldLastErrorSign = 0
                val rigidDampingMult = lerpClamped(
                    FOLLOW_RIGID_REST_HINGE_DAMPING_MULT_SOFT,
                    FOLLOW_RIGID_REST_HINGE_DAMPING_MULT_HARD,
                    rigidRestBlend
                )
                val dampOnlyKd =
                    PhysBearingServoMath.holdKdWithDampingFloor(
                        followHoldKpAlphaRadSec2PerRad,
                        followHoldKdAlphaRadSec,
                        followHoldDampingZetaMin
                    ) * rigidDampingMult * FOLLOW_RIGID_REST_HINGE_MICRO_DAMPING_BOOST
                val holdBiasAlpha =
                    followHoldKpAlphaRadSec2PerRad * followHoldErrorSignedForRest *
                        FOLLOW_RIGID_REST_HINGE_MICRO_HOLD_KP_SCALE
                var microAlpha = holdBiasAlpha - dampOnlyKd * omegaActual
                val snapAlpha = PhysBearingServoMath.computeSnapToZeroAlpha(omegaActual, SERVO_PHYS_TICK_DT_SEC)
                if (snapAlpha * omegaActual < 0.0 && abs(snapAlpha) > abs(microAlpha)) {
                    microAlpha = snapAlpha
                }
                val maxHoldAlpha = followHoldMaxAlphaRadSec2
                    .coerceAtLeast(0.0)
                    .coerceAtMost(SERVO_MAX_ALPHA_HARD)
                val microCap = maxHoldAlpha * FOLLOW_RIGID_REST_HINGE_MICRO_ALPHA_CAP_FRACTION
                alphaCmdRaw = microAlpha.coerceIn(-microCap, microCap)
            } else {
                val rigidDampingMult = lerpClamped(
                    FOLLOW_RIGID_REST_HINGE_DAMPING_MULT_SOFT,
                    FOLLOW_RIGID_REST_HINGE_DAMPING_MULT_HARD,
                    rigidRestBlend
                )
                val dampOnlyKd =
                    PhysBearingServoMath.holdKdWithDampingFloor(
                        followHoldKpAlphaRadSec2PerRad,
                        followHoldKdAlphaRadSec,
                        followHoldDampingZetaMin
                    ) * rigidDampingMult
                val maxHoldAlpha = followHoldMaxAlphaRadSec2
                    .coerceAtLeast(0.0)
                    .coerceAtMost(SERVO_MAX_ALPHA_HARD)
                alphaCmdRaw = (-dampOnlyKd * omegaActual).coerceIn(-maxHoldAlpha, maxHoldAlpha)
            }
            omegaTarget = 0.0
            omegaErr = -omegaActual
        } else {
            tickFollowHingeMicroSuppressed = false
            followHingeMicroTicks = 0
        }

        val followBrakeDecelPhase = followControlPhase == PhysBearingServoMath.FollowPhase.BRAKE_DECEL
        val alphaFilterAlpha = if (followMode && followBrakeDecelPhase) {
            FOLLOW_BRAKE_ALPHA_FILTER_ALPHA
        } else if (followMode && followHoldPhase) {
            FOLLOW_HOLD_ALPHA_FILTER_ALPHA
        } else {
            lerpClamped(SERVO_ALPHA_FILTER_ALPHA_NEAR_HOLD, SERVO_ALPHA_FILTER_ALPHA_ACTIVE, holdBlend)
        }
        servoAlphaCmdFilteredRadSec2 = lowPass(servoAlphaCmdFilteredRadSec2, alphaCmdRaw, alphaFilterAlpha)
        var alphaCmd = if (servoAlphaCmdFilteredRadSec2.isFinite()) {
            servoAlphaCmdFilteredRadSec2
        } else {
            alphaCmdRaw
        }
        if (followMode && followBrakeDecelPhase) {
            alphaCmd = PhysBearingServoMath.clampNoReverse(alphaCmd, omegaActual, SERVO_PHYS_TICK_DT_SEC)
        }
        val jitter = abs(alphaCmdRaw - alphaCmd).let { if (it.isFinite()) it else 0.0 }
        if (jitter > AUTH_JITTER_SOFT_LIMIT) {
            val jitterFactor = 1.0 / (1.0 + (jitter - AUTH_JITTER_SOFT_LIMIT) / AUTH_JITTER_SOFT_LIMIT)
            trackTorqueAuthority = max(AUTH_MIN, trackTorqueAuthority * jitterFactor)
        }
        if (followMode && chainedDynamic) {
            trackTorqueAuthority =
                max(AUTH_MIN, trackTorqueAuthority * followChainTorqueScale.coerceIn(0.0, 1.0))
            if (followHoldPhase) {
                val chainHoldStiffness = followChainStiffnessScale.coerceIn(
                    if (followRigidRestNow) {
                        lerpClamped(FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR, FOLLOW_RIGID_REST_CHAIN_STIFFNESS_FLOOR, rigidRestBlend)
                    } else {
                        FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR
                    },
                    1.0
                )
                holdStabilizerAuthority = max(
                    PhysBearingServoMath.holdRestAuthorityFloor(
                        servoStrength01(),
                        rigidBlend01 = if (followRigidRestNow) rigidRestBlend else 0.0
                    ),
                    holdStabilizerAuthority * chainHoldStiffness
                ).coerceIn(AUTH_MIN, 1.0)
            }
        }
        if (followMode && followControlPhase == PhysBearingServoMath.FollowPhase.TRACK) {
            trackTorqueAuthority = max(trackTorqueAuthority, followTrackAuthorityFloorNow)
        }
        tickTrackTorqueAuthority = trackTorqueAuthority
        tickHoldStabilizerAuthority = holdStabilizerAuthority
        tickFollowModeActive = followMode
        tickFollowBrakePhase = followBrakeDecelPhase
        tickFollowHoldPhase = followHoldPhase

        // Discrete-time stability clamp: limit per-tick omega step to avoid high-frequency solver excitation.
        val maxOmegaStepPerTickBase = lerpClamped(
            SERVO_HOLD_MAX_OMEGA_STEP_PER_TICK,
            SERVO_ACTIVE_MAX_OMEGA_STEP_PER_TICK,
            holdBlend
        )
        val maxOmegaStepPerTick = if (followMode && followControlPhase == PhysBearingServoMath.FollowPhase.TRACK) {
            max(maxOmegaStepPerTickBase, followTrackMaxOmegaStepPerTick)
        } else {
            maxOmegaStepPerTickBase
        }
        val alphaLimitByDt = if (SERVO_PHYS_TICK_DT_SEC > 0.0) {
            maxOmegaStepPerTick / SERVO_PHYS_TICK_DT_SEC
        } else {
            SERVO_MAX_ALPHA_HARD
        }
        val alphaLimit = min(SERVO_MAX_ALPHA_HARD, alphaLimitByDt.coerceAtLeast(0.0))
        alphaCmd = alphaCmd.coerceIn(-alphaLimit, alphaLimit)
        if (followMode && followBrakeDecelPhase) {
            alphaCmd = PhysBearingServoMath.clampNoReverse(alphaCmd, omegaActual, SERVO_PHYS_TICK_DT_SEC)
        }
        if (!alphaCmd.isFinite()) return

        // Ensure the joint isn't treated as free-spin in servo modes. We do NOT rely on the joint motor here because
        // it's backend-dependent; instead we apply an explicit torque servo below.
        var updated = revolute.copy(
            maxForceTorque = computeJointMaxForceTorque(),
            compliance = SERVO_COMPLIANCE,
            driveVelocity = null,
            driveForceLimit = null,
            driveGearRatio = null,
            driveFreeSpin = false
        )
        updated = maybeReanchorJoint(updated, subShip, mainShip, omegaActual)
        if (updated != revolute) {
            joint = updated
            (physLevel as? VsiPhysLevel)?.updateJoint(jointId, updated)
        }

        // Torque servo (works even when revolute drive velocity is not honored).
        val chainBlend = if (chainedDynamic) chainAuthorityBlend01() else 1.0
        val torqueMassMultiplierBase: Double = run {
            val canSub = !subShip.isStatic
            val canMain = mainShip != null && !mainShip.isStatic
            if (!canSub && !canMain) return

            val subInertia = if (canSub) getAngularInertia(subShip, updated.pose0.pos, axisWorld) else 0.0
            val mainInertia = if (canMain) getAngularInertia(mainShip!!, updated.pose1.pos, axisWorld) else 0.0

            when {
                canSub && canMain -> {
                    val baseParallel = parallelOperator(subInertia, mainInertia)
                    if (chainedDynamic) {
                        boostedCoupledEquivalent(baseParallel, subInertia, mainInertia, chainBlend)
                    } else {
                        baseParallel
                    }
                }
                canSub -> subInertia
                else -> mainInertia
            }
        }
        if (!torqueMassMultiplierBase.isFinite() || torqueMassMultiplierBase <= 0.0) {
            appInertiaConsistentTicks = 0
            servoApparentInertiaScale = lowPass(servoApparentInertiaScale, 1.0, SERVO_APPARENT_INERTIA_IDLE_DECAY_ALPHA)
            servoPrevOmegaActualForApparentInertiaScaleRadSec = omegaActualForControl
            servoPrevAppliedHingeTorqueMag = 0.0
            return
        }
        val apparentInertiaScale = if (followMode) {
            appInertiaConsistentTicks = 0
            servoApparentInertiaScale = lowPass(servoApparentInertiaScale, 1.0, SERVO_APPARENT_INERTIA_IDLE_DECAY_ALPHA)
            servoPrevOmegaActualForApparentInertiaScaleRadSec = omegaActualForControl
            1.0
        } else {
            updateApparentInertiaScale(torqueMassMultiplierBase, omegaActualForControl, errorForControl, omegaErr)
        }
        val torqueMassMultiplier = torqueMassMultiplierBase * apparentInertiaScale
        if (!torqueMassMultiplier.isFinite() || torqueMassMultiplier <= 0.0) {
            servoPrevAppliedHingeTorqueMag = 0.0
            return
        }

        // Strong translational lock at the anchors, plus off-axis angular swing lock.
        applyAnchorSeatStabilizer(updated, subShip, mainShip, axisWorld, followAxisHoldProfile)
        applyOffAxisAngularStabilizer(updated, subShip, mainShip, axisWorld, relOmegaWorld, followAxisHoldProfile)

        // Torque servo: tau = I_eff * alpha_cmd.
        var servoTorqueMag = torqueMassMultiplier * alphaCmd
        if (!servoTorqueMag.isFinite()) return
        val baseMaxTorque = if (servoTorqueLimit.isFinite() && servoTorqueLimit > 0.0) servoTorqueLimit else SERVO_TORQUE_MIN
        val torqueAuthority = if (followMode && followHoldPhase) {
            max(trackTorqueAuthority, holdStabilizerAuthority)
        } else {
            trackTorqueAuthority
        }
        var maxTorqueNow = baseMaxTorque * torqueAuthority
        if (gyroRisk) maxTorqueNow *= GYRO_RISK_TORQUE_SCALE
        maxTorqueNow = max(maxTorqueNow, SERVO_TORQUE_MIN)

        servoTorqueMag = servoTorqueMag.coerceIn(-maxTorqueNow, maxTorqueNow)

        // Explicit hinge-axis damping torque for hold/near-target operation:
        // tau_damp = -c * omega, where c = I_eff * damping_rate.
        val nearTargetForDamping = abs(errorForControl) < SERVO_DAMPING_NEAR_TARGET_ERROR_RAD
        val commandNearZero =
            abs(omegaTarget) < SERVO_DAMPING_HOLD_CMD_OMEGA_RAD_SEC &&
                abs(omegaFfRaw) < SERVO_DAMPING_HOLD_CMD_OMEGA_RAD_SEC
        val holdLikeState =
            mode == LockedMode.LOCKED ||
                aligning ||
                (followMode && followHoldPhase) ||
                (!followMode && nearTargetForDamping)
        val applyHingeDamping =
            holdLikeState && commandNearZero && abs(omegaActual) >= SERVO_DAMPING_MIN_ACTIVE_OMEGA_RAD_SEC

        var dampingTorqueMag = 0.0
        if (applyHingeDamping) {
            var dampingRate =
                if (mode == LockedMode.LOCKED || aligning) SERVO_HINGE_DAMPING_RATE_LOCKED else SERVO_HINGE_DAMPING_RATE_NEAR_TARGET
            if (gyroRisk) dampingRate *= 1.2
            val dampingCoeff = torqueMassMultiplier * dampingRate
            dampingTorqueMag = -dampingCoeff * omegaActual
            val maxDampingTorque = min(
                maxTorqueNow * SERVO_HINGE_DAMPING_MAX_TORQUE_FRACTION,
                torqueMassMultiplier * SERVO_HINGE_DAMPING_MAX_ALPHA_EQUIV
            )
            dampingTorqueMag = dampingTorqueMag.coerceIn(-maxDampingTorque, maxDampingTorque)
        }

        val torqueMag = (servoTorqueMag + dampingTorqueMag).coerceIn(-maxTorqueNow, maxTorqueNow)
        if (!torqueMag.isFinite()) {
            servoPrevAppliedHingeTorqueMag = 0.0
            return
        }
        servoPrevAppliedHingeTorqueMag = torqueMag

        val torqueVec = axisWorld.mul(torqueMag, Vector3d())
        if (!subShip.isStatic) {
            subShip.applyWorldTorque(torqueVec)
        }
        if (mainShip != null && !mainShip.isStatic) {
            mainShip.applyWorldTorque(torqueVec.mul(-1.0, Vector3d()))
        }
        if (followMode && followHoldPhase && followHoldSettleTicksRemaining > 0) {
            followHoldSettleTicksRemaining--
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
        if (clientPacket) {
            tag.putBoolean("followAngleStalled", followAngleStalled)
        }

        val mapper = VSJacksonUtil.dtoMapper
        // Avoid saving a potentially large/non-zero drive velocity from mid-servo movement.
        val jointForSave = (joint as? VSRevoluteJoint)?.copy(driveVelocity = null) ?: joint
        if (jointForSave is VSRevoluteJoint) {
            tag.putByteArray("joint", mapper.writeValueAsBytes(jointForSave))
        }

        tag.putInt("jointID", jointID)

        if (shiptraptionID == NO_SHIPTRAPTION_ID) return

        tag.putLong(ClockworkConstants.Nbt.OLD_POS, worldPosition.asLong())
        //to make it more general
        tag.putVector3d(ClockworkConstants.Nbt.OLD_SHIPTRAPTION_CENTER, bearingPos)
        tag.putVector3d(ClockworkConstants.Nbt.NEW_SHIPTRAPTION_CENTER, bearingPos)
    }

    private fun loadTheRest(tag: CompoundTag, level: ServerLevel) {
        val joint = this.joint ?: return
        val mainId = level.getShipManagingPos(worldPosition)?.id

        val oldBPos = BlockPos.of(tag.getLong(ClockworkConstants.Nbt.OLD_POS))
        val oldPos = oldBPos.toJOMLD()

        val newPos = worldPosition.toJOMLD()

        val oldSPos = tag.getVector3d(ClockworkConstants.Nbt.OLD_SHIPTRAPTION_CENTER) ?: return
        val newSPos = tag.getVector3d(ClockworkConstants.Nbt.NEW_SHIPTRAPTION_CENTER) ?: return

        bearingPos = bearingPos.sub(oldSPos).add(newSPos)

        // Worlds may contain legacy fixed-joint NBT. Always convert to a revolute joint on load.
        val revolute = when (joint) {
            is VSRevoluteJoint -> joint
            is VSFixedJoint -> VSRevoluteJoint(
                joint.shipId0, joint.pose0, joint.shipId1, joint.pose1,
                maxForceTorque = computeJointMaxForceTorque(),
                compliance = SERVO_COMPLIANCE,
                driveFreeSpin = true
            )
            else -> throw AssertionError()
        }

        this.joint = revolute.copy(
            shipId0 = shiptraptionID,
            pose0 = VSJointPose(revolute.pose0.pos - oldSPos + newSPos, revolute.pose0.rot),
            shipId1 = mainId,
            pose1 = VSJointPose(revolute.pose1.pos - oldPos + newPos, revolute.pose1.rot),
            driveVelocity = null, // start neutral; physTick will apply servo if needed
        )

        controllerCreationData = PhysBearingData(
            bearingAxis.get(Vector3d()),
            Math.toRadians(targetAngle.toDouble()),
            getRealisticAngularSpeed(),
            (movementMode?.get() ?: LockedMode.UNLOCKED) != LockedMode.UNLOCKED,
            aligning,
            mainId ?: -1L,
            this.joint?.pose1?.pos?.get(Vector3d()) ?: Vector3d(),
            this.joint?.pose0?.pos?.get(Vector3d()) ?: Vector3d()
        )

        tryMakeJoint()
    }

    override fun read(tag: CompoundTag, clientPacket: Boolean) {
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
        if (clientPacket && tag.contains("followAngleStalled")) {
            followAngleStalled = tag.getBoolean("followAngleStalled")
        }

        val mapper = VSJacksonUtil.dtoMapper

        if (tag.contains("constraint")) {
            val temp = mapper.readValue(tag.getByteArray("constraint"), VSJointAndId::class.java)
            joint = temp.joint as VSRevoluteJoint
            jointID = temp.jointId
        } else if (tag.contains("joint")) {
            joint = mapper.readValue(tag.getByteArray("joint"), VSRevoluteJoint::class.java).copy(driveVelocity = null)
            jointID = tag.getInt("jointID")
        } else if (tag.contains("fjoint")) {
            // Legacy worlds: fixed-joint FOLLOW_ANGLE implementation. Convert immediately to revolute.
            val fj = mapper.readValue(tag.getByteArray("fjoint"), VSFixedJoint::class.java)
            joint = VSRevoluteJoint(
                fj.shipId0, fj.pose0, fj.shipId1, fj.pose1,
                maxForceTorque = computeJointMaxForceTorque(),
                compliance = SERVO_COMPLIANCE,
                driveFreeSpin = true
            )
            jointID = tag.getInt("jointID")
        }

        super.read(tag, clientPacket)
        // Behaviours were just read; sync derived servo parameters from the persisted values.
        servoStrengthBehaviour?.also { setServoStrengthSetting(it.currentValue, sendUpdate = false) }
        if (clientPacket) {return}

        // may not have level on read so i have to do this
        loadingFn = { level -> loadTheRest(tag, level) }

        val level = level as? ServerLevel ?: return
        loadingFn!!(level)
        loadingFn = null
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
            if (mode == LockedMode.FOLLOW_ANGLE && followAngleStalled) return 0f
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
        val joint = joint ?: return

        ClockworkMod.physTickOnce(level.dimensionId!!) { level, _, tryNextTick ->
            level as VsiPhysLevel
            if (
                joint.shipId0 != null && level.getShipById(joint.shipId0!!) == null ||
                joint.shipId1 != null && level.getShipById(joint.shipId1!!) == null
            ) {
                if (DEBUG_SERVO) ClockworkMod.LOGGER.info("[PhysBearing] tryMakeJoint: ships not loaded yet; retrying next tick (ship0={}, ship1={}, jointId={})", joint.shipId0, joint.shipId1, jointID)
                tryNextTick()
                return@physTickOnce
            }
            val existing = level.getJointById(jointID)
            if (existing != null) {
                // If the stored jointId is still valid, prefer updating in-place over creating a second joint.
                if (existing is VSRevoluteJoint && joint is VSRevoluteJoint &&
                    existing.shipId0 == joint.shipId0 && existing.shipId1 == joint.shipId1
                ) {
                    level.updateJoint(jointID, joint)
                    isRunning = true
                    return@physTickOnce
                }
                if (existing == joint) {
                    isRunning = true
                    return@physTickOnce
                }
            }
            val id = level.addJoint(joint)
            if (id == -1) {
                tryNextTick()
                return@physTickOnce
            }
            this.jointID = id

            isRunning = true
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
//        val realSpeed = if (getSpeed().absoluteValue > 0.0f) getRealisticAngularSpeed() else 0.0f
//        val newDriveVelocity = if (realSpeed != 0.0f) VSRevoluteJoint.VSRevoluteDriveVelocity(getRealisticAngularSpeed(), true) else null
        joint = VSRevoluteJoint(
            shiptraptionID, VSJointPose(bearingPos.fma(-extraDist, axis, Vector3d()), ship1rot),
            shipOnID, VSJointPose(posInOwnerShip.fma(-extraDist, axis, Vector3d()), ship2rot),
            maxForceTorque = computeJointMaxForceTorque(),
            compliance = SERVO_COMPLIANCE,
            driveFreeSpin = true
//            driveVelocity = newDriveVelocity,
        )

        this.bearingAxis = axis
        this.bearingPos = bearingPos

        controllerCreationData = PhysBearingData(
            bearingAxis.get(Vector3d()),
            Math.toRadians(targetAngle.toDouble()),
            getRealisticAngularSpeed(),
            (movementMode?.get() ?: LockedMode.UNLOCKED) != LockedMode.UNLOCKED,
            aligning,
            shipOnID ?: -1L,
            joint!!.pose1.pos.get(Vector3d()),
            joint!!.pose0.pos.get(Vector3d())
        )

        tryMakeJoint()
        updateDrive()

        sendData()
        updateGeneratedRotation()
    }

    override fun destroy() {
        val level = level ?: return
        if (level.isClientSide || level !is ServerLevel) return

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
            updateDrive()
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
            updateDrive()
            return
        }
        BearingController.getOrCreate(subShip)!!.removePhysBearing(bearingID)

        lastStateChanged = ticks
        resetState()
    }

    private fun resetState() {
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
        servoMode = LockedMode.UNLOCKED
        lockedHoldAngleRad = null
        followOmegaFeedForwardRadSec = 0.0
        lastAligningState = false

        lastAngle = 0f
        curAngle = 0f

        physServoTickCounter = 0
        tickFollowModeActive = false
        tickFollowBrakePhase = false
        tickFollowHoldPhase = false
        tickFollowRestStable = false
        tickFollowRigidRest = false
        tickFollowRigidRestBlend = 0.0
        followWasTrackLastTick = false
        followBrakeHoldAngleRad = null
        clearServoFilterState()
        clearFollowAngleStallState()
    }

    private fun tryAssembleNextTick() {
        if (!assembleNextTick) {return}
        if (ticks - lastStateChanged <= cooldown) {return}
        assembleNextTick = false
        if (!isRunning) {assemble()}
    }

    private fun tryUpdateData() {
        if (shiptraptionID == NO_SHIPTRAPTION_ID) {return}
        val mode = movementMode?.get() ?: LockedMode.UNLOCKED
        val speedNow = getSpeed()
        val aligningNow = aligning

        if (lastSpeed == speedNow && lastMode == mode && lastAligningState == aligningNow) {
            return
        }

        // When entering a hold mode, capture the current physical angle to avoid any instantaneous jump.
        if (lastMode != mode && (mode == LockedMode.FOLLOW_ANGLE || mode == LockedMode.LOCKED)) {
            val shipOn = level!!.getShipObjectManagingPos(blockPos)?.transform
            val shiptraption = level!!.shipObjectWorld.allShips.getById(shiptraptionID)?.transform ?: return
            val currentRad = getAngle(bearingAxis, shiptraption, shipOn)
            targetAngle = Math.toDegrees(currentRad).toFloat()
            lockedHoldAngleRad = if (mode == LockedMode.LOCKED) currentRad else null
        } else if (lastMode == LockedMode.LOCKED && mode != LockedMode.LOCKED) {
            lockedHoldAngleRad = null
        }
        if (lastMode != mode || lastAligningState != aligningNow) {
            clearServoFilterState()
            followBrakeHoldAngleRad = null
            followWasTrackLastTick = false
            tickFollowRestStable = false
            tickFollowRigidRest = false
            tickFollowRigidRestBlend = 0.0
            tickFollowHingeMicroSuppressed = false
            followHingeMicroTicks = 0
            clearFollowAngleStallState()
        }

        lastSpeed = speedNow
        lastMode = mode
        lastAligningState = aligningNow

        updateDrive()
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
            loadingFn?.also {
                it(level as ServerLevel)
                loadingFn = null
            }

            // If the attached sub-ship was deleted (e.g., removed by VS), stop the bearing and let it close.
            // Use allShips (not loadedShips) so chunk load ordering doesn't spuriously disable the bearing.
            if (isRunning && shiptraptionID != NO_SHIPTRAPTION_ID) {
                val sLevel = level as ServerLevel
                val exists = sLevel.shipObjectWorld.allShips.getById(shiptraptionID) != null
                if (!exists) {
                    missingSubShipTicks++
                    if (missingSubShipTicks >= MISSING_SUBSHIP_GRACE_TICKS) {
                        if (DEBUG_SERVO) {
                            ClockworkMod.LOGGER.info("[PhysBearing] subShip missing; disabling (shipId={}, jointId={})", shiptraptionID, jointID)
                        }
                        if (jointID != -1) sLevel.gtpa.removeJoint(jointID)
                        controllerCreationData = null
                        controllerUpdateData = null
                        // Ensure the close animation can run even if we were mid-opening.
                        if (!open && openProgress > 0f) open = true
                        opening = false
                        resetState()
                        tickAnimationLogic()
                        return
                    }
                } else {
                    missingSubShipTicks = 0
                }
            } else {
                missingSubShipTicks = 0
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
        if (mode != LockedMode.FOLLOW_ANGLE && (followAngleStalled || followAngleStallTicks != 0 || lastActualAngleRadForStall != null)) {
            // Stall state is only meaningful for FOLLOW_ANGLE.
            clearFollowAngleStallState()
        }
        if (shiptraptionID == NO_SHIPTRAPTION_ID) {
            targetAngle = 0f
        } else if (joint != null && jointID != -1 && !aligning && mode != LockedMode.LOCKED) {
            val angularSpeed = -getActualAngularSpeed()
            if (!level!!.isClientSide && mode == LockedMode.FOLLOW_ANGLE) {
                updateFollowAngleStallState(angularSpeed)
            }
            if (mode == LockedMode.FOLLOW_ANGLE && followAngleStalled) {
                // Don't advance the expected angle while stalled.
            } else {
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
            updateDrive()
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

    private class ServoSettingValueBoxTransform(private val y: Double, private val x: Double) : ValueBoxTransform.Sided() {
        override fun getSouthLocation(): Vec3 {
            return VecHelper.voxelSpace(x, y, 15.5)
        }

        override fun getLocalOffset(level: LevelAccessor, pos: BlockPos, state: BlockState): Vec3 {
            return super.getLocalOffset(level, pos, state)
                .add(
                    Vec3.atLowerCornerOf(state.getValue(BearingBlock.FACING).normal)
                        .scale((-2 / 16f).toDouble())
                )
        }

        override fun rotate(level: LevelAccessor, pos: BlockPos, state: BlockState, ms: PoseStack) {
            if (!side.axis.isHorizontal) {
                TransformStack.of(ms)
                    .rotateYDegrees((AngleHelper.horizontalAngle(state.getValue(BearingBlock.FACING)) + 180))
            }
            super.rotate(level, pos, state, ms)
        }

        override fun isSideActive(state: BlockState, direction: Direction): Boolean {
            // Match Create's bearing mode selector faces: show on the 4 side faces around the bearing axis.
            return direction.axis != state.getValue(BearingBlock.FACING).axis
        }
    }

    private open class ServoTuningScrollValueBehaviour(
        label: Component,
        be: GeneratingKineticBlockEntity,
        slot: ValueBoxTransform,
        private val behaviourNetId: Int
    ) : ScrollValueBehaviour(label, be, slot) {
        // ScrollValueBehaviour.value is protected; expose it for BE init/sync.
        var currentValue: Int
            get() = value
            set(v) { value = v }

        override fun netId(): Int = behaviourNetId
    }

    private class ServoStrengthScrollValueBehaviour(
        label: Component,
        be: GeneratingKineticBlockEntity,
        slot: ValueBoxTransform
    ) : ServoTuningScrollValueBehaviour(label, be, slot, 1) {

        override fun getType(): BehaviourType<*> = TYPE

        override fun getClipboardKey(): String = "PhysBearingServoStrength"

        override fun write(nbt: CompoundTag, clientPacket: Boolean) {
            nbt.putInt(NBT_KEY, currentValue)
        }

        override fun read(nbt: CompoundTag, clientPacket: Boolean) {
            if (nbt.contains(NBT_KEY)) currentValue = nbt.getInt(NBT_KEY)
        }

        companion object {
            val TYPE: BehaviourType<ServoStrengthScrollValueBehaviour> =
                BehaviourType("phys_bearing_servo_strength")
            private const val NBT_KEY = "ServoStrength"
        }
    }

    companion object {
        const val NO_SHIPTRAPTION_ID: Long = -1

        // Per-bearing tuning UI (Create value boxes).
        private const val SERVO_STRENGTH_MIN = 0
        private const val SERVO_STRENGTH_MAX = 100
        private const val SERVO_STRENGTH_DEFAULT = 50

        // Slider mapping:
        // - Single "Strength" slider controls both stiffness and authority.
        // - Angular control is acceleration-space:
        //     omega_from_error = clamp(wn * angle_error)
        //     alpha_cmd = (2*zeta*wn) * (omega_target - omega_actual)
        //   Then apply torque = I_eff * alpha_cmd.
        private const val SERVO_DAMPING_RATIO_MIN = 1.6 // zeta; >= 1 to reduce sway/overshoot
        private const val SERVO_DAMPING_RATIO_MAX = 2.4
        private const val SERVO_WN_MIN = 2.0
        private const val SERVO_WN_MAX = 24.0
        private const val SERVO_POS_OMEGA_LIMIT_MIN = 3.0
        private const val SERVO_POS_OMEGA_LIMIT_MAX = 28.0

        // Slider output ranges can be globally amplified for testing.
        // This scales all slider-driven magnitudes from the shared strength slider.
        // Hard clamps (omega/alpha/error and seat acceleration/force clamps) remain in effect.
        private const val SERVO_SLIDER_TEST_SCALE = 1.0

        // Strength -> torque limit (log-mapped).
        private const val SERVO_TORQUE_MIN = 1.0e7
        private const val SERVO_TORQUE_MAX = 1.0e10

        private const val MISSING_SUBSHIP_GRACE_TICKS = 20

        // FOLLOW_ANGLE stall detection: if we keep commanding rotation but the actual angle isn't moving,
        // stop advancing the expected angle so we don't push into collisions indefinitely.
        private const val FOLLOW_STALL_TICKS = 8
        private const val FOLLOW_STALL_EPS_RAD_BASE = 1.0e-4
        private const val FOLLOW_STALL_EPS_FRACTION = 4.0e-4
        private const val FOLLOW_STALL_EPS_RAD_MAX = 0.004
        private const val FOLLOW_STALL_CMD_STEP_DEG_PER_TICK = 1.25
        private const val FOLLOW_STALL_CMD_STEP_RATIO = 0.25
        private const val FOLLOW_STALL_CMD_GRACE_TICKS = 6
        private const val FOLLOW_STALL_MAX_OMEGA_FOR_COUNT_RAD_SEC = 0.30
        private const val FOLLOW_STALL_CLEAR_EPS_RAD = 0.003
        private const val FOLLOW_STALL_CLEAR_OMEGA_RAD_SEC = 0.12
        private const val FOLLOW_STALL_CLEAR_MOTION_TICKS = 3

        // Servo tuning.
        private const val SERVO_MAX_OMEGA = 80.0 // rad/s (feed-forward clamp)
        private const val SERVO_MAX_ALPHA_HARD = 180.0 // rad/s^2 (hard stability clamp)
        private const val SERVO_MAX_ERROR_RAD = 0.75 // rad (~43 deg), avoids absurd snap torques on large steps
        private const val SERVO_DEADBAND_RAD = 1.0e-4 // ~0.006 degrees
        private const val SERVO_HOLD_ERROR_RAD = 2.0e-3 // ~0.11 degrees
        private const val SERVO_HOLD_OMEGA_RAD_SEC = 0.04

        // Near hold behavior:
        // - fade stiffness/feed-forward near target so damping can dissipate residual energy
        // - low-pass omega/alpha to avoid exciting high-frequency solver jitter
        // - per-tick omega-step clamp keeps discrete-time response stable at high strength/inertia
        private const val SERVO_PHYS_TICK_DT_SEC = 1.0 / 20.0
        private const val SERVO_HOLD_BLEND_ERROR_RAD = 0.004
        private const val SERVO_HOLD_RELEASE_ERROR_RAD = 0.03
        private const val SERVO_OMEGA_FILTER_ALPHA_NEAR_HOLD = 0.45
        private const val SERVO_OMEGA_FILTER_ALPHA_ACTIVE = 0.80
        private const val SERVO_ALPHA_FILTER_ALPHA_NEAR_HOLD = 0.75
        private const val SERVO_ALPHA_FILTER_ALPHA_ACTIVE = 0.95
        private const val SERVO_HOLD_MAX_OMEGA_STEP_PER_TICK = 0.30
        private const val SERVO_ACTIVE_MAX_OMEGA_STEP_PER_TICK = 1.6
        // Chain attenuation is slider-aware: low slider = more damping bias, high slider = recover full authority.
        private const val SERVO_CHAIN_AUTHORITY_BLEND_START = 0.20
        private const val SERVO_CHAIN_AUTHORITY_BLEND_END = 0.70
        private const val SERVO_CHAIN_COUPLED_EQ_MAX_RATIO = 8.0

        // Authority scheduler (chain/gyro stability)
        private const val AUTH_MIN = 0.18
        private const val AUTH_CHAIN_BASE = 0.55
        private const val AUTH_OFFAXIS_SOFT_LIMIT = 1.2 // rad/s
        private const val AUTH_RELVEL_SOFT_LIMIT = 0.6 // m/s
        private const val AUTH_JITTER_SOFT_LIMIT = 40.0 // rad/s^2 difference between raw and filtered alpha
        private const val FOLLOW_TRACK_AUTH_GYRO_FLOOR_SCALE = 0.65
        private const val OMEGA_EMA_ALPHA = 0.35 // omega EMA smoothing

        // Apparent inertia compensation:
        // Estimate effective hinge inertia from previous applied torque vs observed omega response, then scale
        // authority up for downstream chained loads so nested ships behave closer to one rigid assembly.
        private const val SERVO_APPARENT_INERTIA_MAX_SCALE_MIN_STRENGTH = 2.0
        private const val SERVO_APPARENT_INERTIA_MAX_SCALE_MAX_STRENGTH = 16.0
        private const val SERVO_APPARENT_INERTIA_STABILIZER_SHARE = 0.50
        private const val SERVO_APPARENT_INERTIA_STABILIZER_MAX_SCALE = 6.0
        private const val SERVO_APPARENT_INERTIA_MIN_BASE = 1.0e-3
        private const val SERVO_APPARENT_INERTIA_MIN_OBS_TORQUE = 1.0e6
        private const val SERVO_APPARENT_INERTIA_ALPHA_FLOOR_RAD_SEC2 = 0.20
        private const val SERVO_APPARENT_INERTIA_ACTIVE_ERROR_RAD = 0.015
        private const val SERVO_APPARENT_INERTIA_ACTIVE_OMEGA_ERR_RAD_SEC = 0.40
        private const val SERVO_APPARENT_INERTIA_RISE_ALPHA = 0.30
        private const val SERVO_APPARENT_INERTIA_FALL_ALPHA = 0.06
        private const val SERVO_APPARENT_INERTIA_OPPOSED_DECAY_ALPHA = 0.05
        private const val SERVO_APPARENT_INERTIA_IDLE_DECAY_ALPHA = 0.02
        private const val SERVO_APPARENT_INERTIA_SETTLED_ERROR_RAD = 0.01
        private const val SERVO_APPARENT_INERTIA_SETTLED_OMEGA_RAD_SEC = 0.10
        private const val SERVO_APPARENT_INERTIA_SETTLED_DECAY_ALPHA = 0.12

        // Apparent inertia estimator stability
        private const val APP_INERTIA_DEBOUNCE_TICKS = 3
        private const val APP_INERTIA_MAX_RISE_PER_TICK_CHAIN = 1.04
        private const val APP_INERTIA_MAX_RISE_PER_TICK = 1.08
        private const val APP_INERTIA_MIN_FALL_PER_TICK = 0.98

        // Gyro safety detection thresholds (heuristics)
        private const val GYRO_RISK_OFFAXIS_OMEGA = 1.2 // rad/s
        private const val GYRO_RISK_HINGE_OMEGA = 14.0 // rad/s
        private const val GYRO_RISK_RELVEL = 1.0 // m/s
        private const val GYRO_RISK_TORQUE_SCALE = 0.5

        // Hinge-axis explicit damping: tau_damp = -c * omega (c = I_eff * damping_rate).
        // Applied in LOCKED/aligning or when close to target with near-zero command.
        private const val SERVO_DAMPING_NEAR_TARGET_ERROR_RAD = 0.03
        private const val SERVO_DAMPING_HOLD_CMD_OMEGA_RAD_SEC = 0.20
        private const val SERVO_DAMPING_MIN_ACTIVE_OMEGA_RAD_SEC = 0.03
        private const val SERVO_HINGE_DAMPING_RATE_LOCKED = 1.4
        private const val SERVO_HINGE_DAMPING_RATE_NEAR_TARGET = 1.0
        private const val SERVO_HINGE_DAMPING_MAX_TORQUE_FRACTION = 0.12
        private const val SERVO_HINGE_DAMPING_MAX_ALPHA_EQUIV = 8.0

        // FOLLOW_ANGLE control phases:
        // - TRACK while kinetic command is active.
        // - BRAKE_DECEL when command is near zero or stalled.
        // - BRAKE_HOLD after low-speed capture.
        private const val FOLLOW_CMD_EPS_RAD_SEC = 1.0e-3
        private const val FOLLOW_BRAKE_HOLD_CAPTURE_OMEGA_RAD_SEC = 0.08
        private const val FOLLOW_HOLD_CAPTURE_TICKS = 3
        private const val FOLLOW_HOLD_EMERGENCY_RELEASE_OMEGA_RAD_SEC = 1.8
        private const val FOLLOW_HOLD_EMERGENCY_RELEASE_TICKS = 4
        private const val FOLLOW_BRAKE_ALPHA_FILTER_ALPHA = 1.0
        private const val FOLLOW_CHAIN_STABILIZER_DAMP_BOOST = 0.70
        private const val FOLLOW_HOLD_ERROR_DEADBAND_RAD = 0.0010
        private const val FOLLOW_HOLD_OMEGA_DEADBAND_RAD_SEC = 0.006
        private const val FOLLOW_HOLD_VEL_KILL_ERROR_BAND_RAD = 0.020
        private const val FOLLOW_HOLD_VEL_KILL_OMEGA_MAX_RAD_SEC = 0.35
        private const val FOLLOW_HOLD_RINGDOWN_ERROR_MIN_RAD = 0.005
        private const val FOLLOW_HOLD_RINGDOWN_OMEGA_MIN_RAD_SEC = 0.08
        private const val FOLLOW_HOLD_RINGDOWN_TICKS = 8
        private const val FOLLOW_REST_STABLE_ENTER_TICKS = 4
        private const val FOLLOW_REST_STABLE_HOLD_ERROR_ENTER_RAD = 0.004
        private const val FOLLOW_REST_STABLE_HOLD_ERROR_EXIT_RAD = 0.010
        private const val FOLLOW_REST_STABLE_HINGE_OMEGA_ENTER_RAD_SEC = 0.05
        private const val FOLLOW_REST_STABLE_HINGE_OMEGA_EXIT_RAD_SEC = 0.16
        private const val FOLLOW_REST_STABLE_OFFAXIS_OMEGA_ENTER_RAD_SEC = 0.07
        private const val FOLLOW_REST_STABLE_OFFAXIS_OMEGA_EXIT_RAD_SEC = 0.20
        private const val FOLLOW_REST_STABLE_BAND_SCALE_MIN = 0.55
        private const val FOLLOW_REST_ULTRA_STABLE_ENTER_TICKS = 8
        private const val FOLLOW_REST_BIAS_ZERO_BAND_SEAT = 0.10
        private const val FOLLOW_REST_BIAS_ZERO_BAND_TILT = 0.20
        private const val FOLLOW_HOLD_AUTH_MIN = 0.42
        private const val FOLLOW_HOLD_BIAS_ACTIVE_RATIO = 0.08
        private const val FOLLOW_HOLD_BIAS_CORRECTIVE_DOT_THRESHOLD = 0.01
        private const val FOLLOW_HOLD_HEAVY_LEVER_BLEND_START_M = 1.5
        private const val FOLLOW_HOLD_HEAVY_LEVER_BLEND_END_M = 8.0
        private const val FOLLOW_HOLD_HEAVY_SEAT_KP_SCALE_MIN = 0.62
        private const val FOLLOW_HOLD_HEAVY_SEAT_KD_BOOST_MAX = 2.80
        private const val FOLLOW_HOLD_HEAVY_SEAT_ACCEL_CAP_SCALE_MAX = 3.20
        private const val FOLLOW_HOLD_HEAVY_SEAT_BIAS_CAP_SCALE_MAX = 3.00
        private const val FOLLOW_HOLD_HEAVY_TILT_KP_SCALE_MIN = 0.50
        private const val FOLLOW_HOLD_HEAVY_TILT_KD_BOOST_MAX = 3.20
        private const val FOLLOW_HOLD_HEAVY_TILT_ALPHA_CAP_SCALE_MAX = 2.80
        private const val FOLLOW_HOLD_HEAVY_TILT_ALPHA_EQ_CAP_SCALE_MAX = 2.60
        private const val FOLLOW_HOLD_HEAVY_TILT_BIAS_CAP_SCALE_MAX = 3.20
        private const val FOLLOW_HOLD_CHAIN_STIFFNESS_FLOOR = 0.45
        private const val FOLLOW_RIGID_REST_CHAIN_STIFFNESS_FLOOR = 0.72
        private const val FOLLOW_RIGID_REST_BLEND_START = 0.55
        private const val FOLLOW_RIGID_REST_BLEND_END = 1.00
        private const val FOLLOW_RIGID_REST_ENTER_TICKS_SOFT = 10
        private const val FOLLOW_RIGID_REST_ENTER_TICKS_HARD = 6
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_ENTER_SOFT_RAD = 0.0016
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_ENTER_HARD_RAD = 0.0030
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_EXIT_SOFT_RAD = 0.0060
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_EXIT_HARD_RAD = 0.0100
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_ENTER_SOFT_RAD_SEC = 0.018
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_ENTER_HARD_RAD_SEC = 0.035
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_EXIT_SOFT_RAD_SEC = 0.080
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_EXIT_HARD_RAD_SEC = 0.130
        private const val FOLLOW_RIGID_REST_OFFAXIS_OMEGA_ENTER_SOFT_RAD_SEC = 0.030
        private const val FOLLOW_RIGID_REST_OFFAXIS_OMEGA_ENTER_HARD_RAD_SEC = 0.055
        private const val FOLLOW_RIGID_REST_OFFAXIS_OMEGA_EXIT_SOFT_RAD_SEC = 0.120
        private const val FOLLOW_RIGID_REST_OFFAXIS_OMEGA_EXIT_HARD_RAD_SEC = 0.180
        private const val FOLLOW_RIGID_REST_RELVEL_ENTER_SOFT_MPS = 0.015
        private const val FOLLOW_RIGID_REST_RELVEL_ENTER_HARD_MPS = 0.030
        private const val FOLLOW_RIGID_REST_RELVEL_EXIT_SOFT_MPS = 0.060
        private const val FOLLOW_RIGID_REST_RELVEL_EXIT_HARD_MPS = 0.100
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_BAND_SOFT_RAD = 0.0015
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_BAND_HARD_RAD = 0.0025
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_BAND_EXIT_SOFT_RAD = 0.0038
        private const val FOLLOW_RIGID_REST_HINGE_ERROR_BAND_EXIT_HARD_RAD = 0.0062
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_SOFT_RAD_SEC = 0.012
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_HARD_RAD_SEC = 0.020
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_EXIT_SOFT_RAD_SEC = 0.028
        private const val FOLLOW_RIGID_REST_HINGE_OMEGA_BAND_EXIT_HARD_RAD_SEC = 0.050
        private const val FOLLOW_RIGID_REST_HINGE_MICRO_ENTER_TICKS_SOFT = 4
        private const val FOLLOW_RIGID_REST_HINGE_MICRO_ENTER_TICKS_HARD = 2
        private const val FOLLOW_RIGID_REST_HINGE_DAMPING_MULT_SOFT = 1.05
        private const val FOLLOW_RIGID_REST_HINGE_DAMPING_MULT_HARD = 1.25
        private const val FOLLOW_RIGID_REST_HINGE_MICRO_DAMPING_BOOST = 1.35
        private const val FOLLOW_RIGID_REST_HINGE_MICRO_HOLD_KP_SCALE = 0.30
        private const val FOLLOW_RIGID_REST_HINGE_MICRO_ALPHA_CAP_FRACTION = 0.45
        private const val FOLLOW_HOLD_ALPHA_FILTER_ALPHA = 0.92
        private const val FOLLOW_HOLD_SEAT_VEL_SUPPRESSION_FLOOR = 0.75
        private const val FOLLOW_HOLD_SEAT_KP_MULT = 1.15
        private const val FOLLOW_HOLD_SEAT_KD_MULT = 1.35
        private const val FOLLOW_HOLD_TILT_KP_MULT = 1.20
        private const val FOLLOW_HOLD_TILT_KD_MULT = 1.35
        private const val FOLLOW_REST_STABLE_SEAT_ERROR_BAND_M = 0.003
        private const val FOLLOW_REST_STABLE_SEAT_VEL_BAND_MPS = 0.04
        private const val FOLLOW_HOLD_MICRO_SEAT_ENTER_TICKS = 3
        private const val FOLLOW_HOLD_MICRO_SEAT_ERROR_ENTER_M = 0.012
        private const val FOLLOW_HOLD_MICRO_SEAT_VEL_ENTER_MPS = 0.16
        private const val FOLLOW_HOLD_MICRO_SEAT_ERROR_EXIT_M = 0.024
        private const val FOLLOW_HOLD_MICRO_SEAT_VEL_EXIT_MPS = 0.30
        private const val FOLLOW_HOLD_MICRO_SEAT_KP_SCALE = 0.58
        private const val FOLLOW_HOLD_MICRO_SEAT_KD_MULT = 1.35
        private const val FOLLOW_RIGID_REST_MICRO_SEAT_KP_SCALE = 0.42
        private const val FOLLOW_RIGID_REST_MICRO_SEAT_KD_MULT = 1.85
        private const val FOLLOW_REST_STABLE_TILT_OMEGA_BAND_RAD_SEC = 0.07
        private const val FOLLOW_REST_STABLE_TILT_ERROR_BAND_RAD = 0.004
        private const val FOLLOW_HOLD_MICRO_TILT_ENTER_TICKS = 3
        private const val FOLLOW_HOLD_MICRO_TILT_ERROR_ENTER_RAD = 0.022
        private const val FOLLOW_HOLD_MICRO_TILT_OMEGA_ENTER_RAD_SEC = 0.22
        private const val FOLLOW_HOLD_MICRO_TILT_ERROR_EXIT_RAD = 0.040
        private const val FOLLOW_HOLD_MICRO_TILT_OMEGA_EXIT_RAD_SEC = 0.36
        private const val FOLLOW_HOLD_MICRO_TILT_KP_SCALE = 0.56
        private const val FOLLOW_HOLD_MICRO_TILT_KD_MULT = 1.30
        private const val FOLLOW_RIGID_REST_MICRO_TILT_KP_SCALE = 0.38
        private const val FOLLOW_RIGID_REST_MICRO_TILT_KD_MULT = 1.80
        private const val FOLLOW_HOLD_EXTRA_OFFAXIS_DAMPING_RAD_SEC = 0.75
        private const val FOLLOW_HOLD_TILT_STIFFNESS_FLOOR = 0.30
        private const val FOLLOW_BRAKE_TILT_MAX_ALPHA_MULT = 1.20
        private const val FOLLOW_HOLD_TILT_MAX_ALPHA_MULT = 1.35
        private const val FOLLOW_HOLD_SETTLE_SEAT_VEL_BLEND_START_MPS = 0.03
        private const val FOLLOW_HOLD_SETTLE_SEAT_VEL_BLEND_END_MPS = 0.30
        private const val FOLLOW_HOLD_SETTLE_SEAT_KP_SCALE_MIN = 0.75
        private const val FOLLOW_HOLD_SETTLE_SEAT_KD_BOOST_MAX = 1.65
        private const val FOLLOW_HOLD_SETTLE_TILT_OMEGA_BLEND_START_RAD_SEC = 0.04
        private const val FOLLOW_HOLD_SETTLE_TILT_OMEGA_BLEND_END_RAD_SEC = 0.40
        private const val FOLLOW_HOLD_SETTLE_TILT_KP_SCALE_MIN = 0.55
        private const val FOLLOW_HOLD_SETTLE_TILT_KD_BOOST_MAX = 2.20
        private const val FOLLOW_HOLD_SETTLE_VERTICAL_FLOOR_SCALE_MIN = 0.45
        private const val FOLLOW_HOLD_SETTLE_FLOOR_OMEGA_BLEND_START_RAD_SEC = 0.05
        private const val FOLLOW_HOLD_SETTLE_FLOOR_OMEGA_BLEND_END_RAD_SEC = 0.60
        private const val FOLLOW_HOLD_SETTLE_MOTION_FLOOR_SCALE_MIN = 0.35
        private const val FOLLOW_HOLD_SETTLE_MIN_STIFFNESS_FLOOR = 0.08

        // Off-axis anchor seating (perpendicular to hinge axis) to suppress lateral wobble under heavy load.
        // Shared strength slider maps both seat stiffness and seat force limit.
        private const val SERVO_SEAT_DAMPING_RATIO_MIN = 1.4
        private const val SERVO_SEAT_DAMPING_RATIO_MAX = 2.8
        private const val SERVO_SEAT_WN_MIN = 2.0
        private const val SERVO_SEAT_WN_MAX = 20.0
        private const val SERVO_SEAT_MAX_ERROR_M = 0.35
        private const val SERVO_SEAT_MAX_ACCEL_HARD = 40.0 // m/s^2 hard stability clamp
        private const val SERVO_SEAT_MAX_ACCEL_WORLD_ANCHOR = 8.0
        private const val SERVO_SEAT_WORLD_ANCHOR_KP_SCALE = 0.35
        private const val SERVO_SEAT_WORLD_ANCHOR_KD_SCALE = 0.65
        private const val SERVO_SEAT_MAX_ACCEL_CHAIN = 12.0
        private const val SERVO_SEAT_CHAIN_WN_SCALE = 0.90
        private const val SERVO_SEAT_CHAIN_KP_SCALE = 0.60
        private const val SERVO_SEAT_CHAIN_KD_SCALE = 1.00
        private const val SERVO_SEAT_CHAIN_ERROR_DEADBAND_M = 1.2e-3
        private const val SERVO_SEAT_CHAIN_VEL_DEADBAND = 0.03
        private const val SERVO_SEAT_ERROR_DEADBAND_M = 1.0e-3
        private const val SERVO_SEAT_VEL_DEADBAND = 0.02
        private const val SERVO_SEAT_FORCE_LIMIT_MIN = 5.0e5
        private const val SERVO_SEAT_FORCE_LIMIT_MAX = 2.0e10

        // Off-axis angular swing lock (suppresses metronome-like side swing in bearing chains).
        private const val SERVO_TILT_OMEGA_DEADBAND = 2.0e-3 // rad/s
        private const val SERVO_TILT_DAMPING_RATIO_MIN = 1.8
        private const val SERVO_TILT_DAMPING_RATIO_MAX = 3.0
        private const val SERVO_TILT_WN_MIN = 6.0
        private const val SERVO_TILT_WN_MAX = 60.0
        private const val SERVO_TILT_MAX_ERROR_RAD = 0.4
        private const val SERVO_TILT_DAMP_ONLY_ERROR_RAD = 0.01
        private const val SERVO_TILT_FULL_STIFFNESS_ERROR_RAD = 0.12
        private const val SERVO_TILT_CHAIN_DAMP_ONLY_ERROR_RAD = 0.012
        private const val SERVO_TILT_CHAIN_FULL_STIFFNESS_ERROR_RAD = 0.10
        private const val SERVO_TILT_CHAIN_OMEGA_DEADBAND = 0.006
        private const val SERVO_TILT_CHAIN_WN_SCALE = 0.55
        private const val SERVO_TILT_CHAIN_KP_SCALE = 0.85
        private const val SERVO_TILT_CHAIN_KD_SCALE = 1.10
        private const val SERVO_TILT_WORLD_ANCHOR_WN_SCALE = 0.35
        private const val SERVO_TILT_WORLD_ANCHOR_KD_SCALE = 1.15
        private const val SERVO_TILT_MAX_ALPHA_CHAIN = 60.0
        private const val SERVO_TILT_MAX_ALPHA_WORLD_ANCHOR = 20.0
        private const val SERVO_TILT_MAX_ALPHA_HARD = 200.0 // rad/s^2 hard stability clamp
        private const val SERVO_TILT_MAX_ALPHA_EQUIV = 12.0
        private const val SERVO_TILT_MAX_ALPHA_EQUIV_WORLD_ANCHOR = 4.0
        private const val SERVO_TILT_MAX_ALPHA_EQUIV_CHAIN = 7.0
        private const val SERVO_TILT_TORQUE_MIN = 1.0e6
        private const val SERVO_TILT_TORQUE_MAX = 5.0e11

        // Joint constraint strength (keeps the bearing locked into the rotation plane).
        private const val SERVO_JOINT_MAX_FORCE = 1.0e10f
        private const val SERVO_JOINT_ANCHOR_OFFSET = 1.0

        // Gentle re-anchoring to correct accumulated joint drift (does not retarget angle/pose each tick).
        private const val SERVO_REANCHOR_PERIOD_TICKS = 30
        private const val SERVO_REANCHOR_POS_EPS = 0.12
        private const val SERVO_REANCHOR_MAX_STEP = 0.03
        private const val SERVO_REANCHOR_MAX_HINGE_OMEGA_RAD_SEC = 0.25
        private const val SERVO_REANCHOR_MAX_REL_VEL_MPS = 0.12
        private const val SERVO_REANCHOR_MIN_MOTION_SCALE = 0.20
        private const val SERVO_REANCHOR_MIN_STEP = 0.003
        private const val SERVO_REANCHOR_CHAIN_MAX_HINGE_OMEGA_RAD_SEC = 0.10
        private const val SERVO_REANCHOR_CHAIN_MAX_REL_VEL_MPS = 0.05
        private const val SERVO_REANCHOR_CHAIN_STEP_SCALE = 0.45

        // Extremely stiff, but not pathological (avoid ~1e-100 which can blow up solvers).
        private const val SERVO_COMPLIANCE = 1.0e-10

        private const val DEBUG_SERVO = false

        //tolerance is in degrees
        @JvmStatic
        fun canDisassemble(bearingAxis: Vector3d, mainShip: ServerShip, otherShip: ServerShip?, tolerance: Int=5): Boolean {
            if (abs(Math.toDegrees(getAngle(bearingAxis, mainShip.transform, otherShip?.transform))) > tolerance) return false
            return true
        }
    }
}
