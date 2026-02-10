package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import org.joml.Vector3d
import org.valkyrienskies.clockwork.platform.api.ContraptionController.LockedMode

internal object PhysBearingFollowController {
    data class TickSnapshot(
        val currentAngleRad: Double,
        val targetAngleRad: Double,
        val omegaActualRadSec: Double,
        val offAxisOmegaRadSec: Double,
        val relAnchorSpeedMps: Double,
        val axisWorld: Vector3d,
        val chainedDynamic: Boolean,
        val gyroRisk: Boolean
    )

    data class PhaseRequest(
        val followAngleStalled: Boolean,
        val commandOmegaRadSec: Double,
        val omegaActualRadSec: Double,
        val previousWasTrack: Boolean,
        val holdLatchedAngle: Double?,
        val captureTicks: Int,
        val emergencyTicks: Int,
        val captureOmegaRadSec: Double,
        val captureTicksRequired: Int,
        val emergencyReleaseOmegaRadSec: Double,
        val emergencyReleaseTicksRequired: Int,
        val commandActiveEpsilonRadSec: Double
    )

    data class PhaseResult(
        val phase: PhysBearingServoMath.FollowPhase,
        val holdPhase: Boolean,
        val trackPhase: Boolean,
        val enteredHoldFromDecel: Boolean,
        val holdLatched: Boolean,
        val captureTicks: Int,
        val emergencyTicks: Int
    )

    fun isLockedFixedMode(isLockedMode: Boolean, aligning: Boolean): Boolean {
        return isLockedMode && !aligning
    }

    fun isLockedFixedMode(mode: LockedMode, aligning: Boolean): Boolean {
        return isLockedFixedMode(mode == LockedMode.LOCKED, aligning)
    }

    fun normalizedSnapDeltaRad(targetAngleRad: Double, currentAngleRad: Double): Double {
        if (!targetAngleRad.isFinite() || !currentAngleRad.isFinite()) return 0.0
        val d = targetAngleRad - currentAngleRad
        return atan2(sin(d), cos(d))
    }

    fun evolvePhase(request: PhaseRequest): PhaseResult {
        val cmdOmega = if (request.followAngleStalled) 0.0 else request.commandOmegaRadSec
        val trackActive =
            !request.followAngleStalled &&
                PhysBearingServoMath.isFollowCommandActive(
                    commandAbsOmegaRadSec = abs(cmdOmega),
                    epsilonRadSec = request.commandActiveEpsilonRadSec
                )

        val previousPhase = when {
            request.previousWasTrack -> PhysBearingServoMath.FollowPhase.TRACK
            request.holdLatchedAngle != null -> PhysBearingServoMath.FollowPhase.BRAKE_HOLD
            else -> PhysBearingServoMath.FollowPhase.BRAKE_DECEL
        }

        val phaseState = PhysBearingServoMath.stepFollowPhase(
            trackActive = trackActive,
            omegaAbsRadSec = abs(request.omegaActualRadSec),
            previous = PhysBearingServoMath.FollowPhaseState(
                phase = previousPhase,
                holdLatched = request.holdLatchedAngle != null,
                captureTicks = request.captureTicks,
                emergencyTicks = request.emergencyTicks
            ),
            captureOmegaRadSec = request.captureOmegaRadSec,
            captureTicksRequired = request.captureTicksRequired,
            emergencyReleaseOmegaRadSec = request.emergencyReleaseOmegaRadSec,
            emergencyReleaseTicksRequired = request.emergencyReleaseTicksRequired
        )

        val phase = phaseState.phase
        return PhaseResult(
            phase = phase,
            holdPhase = phase == PhysBearingServoMath.FollowPhase.BRAKE_HOLD,
            trackPhase = phase == PhysBearingServoMath.FollowPhase.TRACK,
            enteredHoldFromDecel = previousPhase == PhysBearingServoMath.FollowPhase.BRAKE_DECEL && phase == PhysBearingServoMath.FollowPhase.BRAKE_HOLD,
            holdLatched = phaseState.holdLatched,
            captureTicks = phaseState.captureTicks,
            emergencyTicks = phaseState.emergencyTicks
        )
    }

    fun clearModeRuntime(state: PhysBearingFollowState) {
        state.followWasTrackLastTick = false
        state.followBrakeHoldAngleRad = null
        state.followHoldSettleTicksRemaining = 0
        state.followHoldAcquireTicksRemaining = 0
        state.followHoldCaptureTicks = 0
        state.followHoldEmergencyReleaseTicks = 0
        state.followHoldRingDownTicks = 0
        state.followHoldLastErrorSign = 0
        state.followRestStableTicks = 0
        state.followRestUltraStableTicks = 0
        state.followUltraSleepTicks = 0
        state.followRigidRestTicks = 0
        state.followHingeMicroTicks = 0
        state.followSeatMicroTicks = 0
        state.followTiltMicroTicks = 0

        state.tickFollowRestStable = false
        state.tickFollowRestUltraStable = false
        state.tickFollowUltraSleep = false
        state.tickFollowRigidRest = false
        state.tickFollowHingeMicroSuppressed = false
        state.tickFollowSeatMicroSuppressed = false
        state.tickFollowTiltMicroSuppressed = false
        state.tickFollowSeatRestBandSatisfied = false
        state.tickFollowTiltRestBandSatisfied = false

        state.followSeatHoldBiasAccWorld = Vector3d()
        state.followTiltHoldBiasAlphaWorld = Vector3d()
        state.followSeatBiasFreezeTicks = 0
        state.followTiltBiasFreezeTicks = 0
        state.followPrevSeatBiasErrorWorld = Vector3d()
        state.followPrevTiltBiasErrorWorld = Vector3d()
    }
}
