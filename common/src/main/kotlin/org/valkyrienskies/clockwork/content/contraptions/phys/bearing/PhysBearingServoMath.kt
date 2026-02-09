package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sqrt

internal object PhysBearingServoMath {
    enum class FollowPhase {
        TRACK,
        BRAKE_DECEL,
        BRAKE_HOLD
    }

    data class FollowPhaseState(
        val phase: FollowPhase,
        val holdLatched: Boolean,
        val captureTicks: Int,
        val emergencyTicks: Int
    )

    data class RingDownState(
        val ticks: Int,
        val errorSign: Int
    )

    data class HysteresisLatchState(
        val active: Boolean,
        val ticks: Int
    )

    data class FollowStallState(
        val stalled: Boolean,
        val stallTicks: Int,
        val clearTicks: Int,
        val graceTicks: Int
    )

    data class AxisHoldProfile(
        val verticalBlend: Double,
        val horizontalBlend: Double,
        val seatKpScale: Double,
        val seatKdScale: Double,
        val tiltKpScale: Double,
        val tiltKdScale: Double,
        val restGateScale: Double,
        val extraOffAxisDamping: Double,
        val worldSeatAccelCapScale: Double,
        val worldTiltAlphaCapScale: Double,
        val worldTiltAlphaEqCapScale: Double,
        val microKpFloor: Double,
        val microKdBoost: Double,
        val offAxisDampingZetaMin: Double
    ) {
        companion object {
            val IDENTITY = AxisHoldProfile(
                verticalBlend = 0.0,
                horizontalBlend = 0.0,
                seatKpScale = 1.0,
                seatKdScale = 1.0,
                tiltKpScale = 1.0,
                tiltKdScale = 1.0,
                restGateScale = 1.0,
                extraOffAxisDamping = 0.0,
                worldSeatAccelCapScale = 1.0,
                worldTiltAlphaCapScale = 1.0,
                worldTiltAlphaEqCapScale = 1.0,
                microKpFloor = 1.0,
                microKdBoost = 1.0,
                offAxisDampingZetaMin = 1.0
            )
        }
    }

    data class FollowStrengthParams(
        val stopTimeSec: Double,
        val brakeRate: Double,
        val brakeStaticAlpha: Double,
        val maxBrakeAlpha: Double,
        val trackOmegaGain: Double,
        val trackPosAssistGain: Double,
        val trackPosAssistOmegaLimit: Double,
        val trackAuthorityFloor: Double,
        val trackMaxOmegaStepPerTick: Double,
        val holdKpAlpha: Double,
        val holdKdAlpha: Double,
        val holdDampingZetaMin: Double,
        val holdRingDownKdBoost: Double,
        val holdMaxAlpha: Double,
        val brakeSeatKpScale: Double,
        val brakeSeatKdScale: Double,
        val brakeTiltKpScale: Double,
        val brakeTiltKdScale: Double,
        val brakeTiltStiffnessFloor: Double,
        val brakeTiltDeadbandScale: Double,
        val chainTorqueScale: Double,
        val chainStiffnessScale: Double,
        val holdWorldSeatKpBoost: Double,
        val holdWorldSeatKdBoost: Double,
        val holdWorldTiltKpBoost: Double,
        val holdWorldTiltKdBoost: Double,
        val holdWorldTiltAlphaCapScale: Double,
        val holdWorldTiltAlphaEqCapScale: Double,
        val holdRestTiltStiffnessFloor: Double,
        val verticalHoldSeatKpScale: Double,
        val verticalHoldSeatKdScale: Double,
        val verticalHoldTiltKpScale: Double,
        val verticalHoldTiltKdScale: Double,
        val verticalHoldExtraOffAxisDamping: Double,
        val verticalHoldRestGateScale: Double,
        val verticalHoldWorldSeatAccelCapScale: Double,
        val verticalHoldWorldTiltAlphaCapScale: Double,
        val verticalHoldWorldTiltAlphaEqCapScale: Double,
    )

    fun mapFollowStrength(strength01: Double, sliderScale: Double): FollowStrengthParams {
        val t = strength01.coerceIn(0.0, 1.0)
        val scale = sliderScale.coerceAtLeast(0.0)

        val stopTimeSec = lerpLog(FOLLOW_STOP_TIME_MAX_SEC, FOLLOW_STOP_TIME_MIN_SEC, t)
        val brakeRate = (ln(FOLLOW_STOP_SETTLE_RATIO) / stopTimeSec) * scale
        val brakeStaticAlpha = lerpLog(FOLLOW_STATIC_BRAKE_ALPHA_MIN, FOLLOW_STATIC_BRAKE_ALPHA_MAX, t) * scale
        val maxBrakeAlpha = lerpLog(FOLLOW_MAX_BRAKE_ALPHA_MIN, FOLLOW_MAX_BRAKE_ALPHA_MAX, t) * scale
        val trackOmegaGain = lerpLog(FOLLOW_TRACK_OMEGA_GAIN_MIN, FOLLOW_TRACK_OMEGA_GAIN_MAX, t) * scale
        val trackPosAssistGain = lerpLog(FOLLOW_TRACK_POS_GAIN_MIN, FOLLOW_TRACK_POS_GAIN_MAX, t) * scale
        val trackPosAssistOmegaLimit = lerpLog(FOLLOW_TRACK_POS_OMEGA_LIMIT_MIN, FOLLOW_TRACK_POS_OMEGA_LIMIT_MAX, t) * scale
        val trackAuthorityFloor = lerpClamped(FOLLOW_TRACK_AUTH_FLOOR_MIN, FOLLOW_TRACK_AUTH_FLOOR_MAX, t).coerceIn(0.0, 1.0)
        val trackMaxOmegaStepPerTick =
            lerpClamped(FOLLOW_TRACK_MAX_OMEGA_STEP_PER_TICK_MIN, FOLLOW_TRACK_MAX_OMEGA_STEP_PER_TICK_MAX, t)
                .coerceAtLeast(0.0)

        val holdKpAlpha = lerpLog(FOLLOW_HOLD_KP_ALPHA_MIN, FOLLOW_HOLD_KP_ALPHA_MAX, t) * scale
        val holdKdAlpha = lerpLog(FOLLOW_HOLD_KD_ALPHA_MIN, FOLLOW_HOLD_KD_ALPHA_MAX, t) * scale
        val holdDampingZetaMin = lerpClamped(FOLLOW_HOLD_DAMPING_ZETA_MIN, FOLLOW_HOLD_DAMPING_ZETA_MAX, t)
        val holdRingDownKdBoost = lerpClamped(FOLLOW_HOLD_RINGDOWN_KD_BOOST_MIN, FOLLOW_HOLD_RINGDOWN_KD_BOOST_MAX, t)
        val holdMaxAlpha = lerpLog(FOLLOW_HOLD_MAX_ALPHA_MIN, FOLLOW_HOLD_MAX_ALPHA_MAX, t) * scale

        val brakeSeatKpScale = lerpClamped(FOLLOW_BRAKE_SEAT_KP_SCALE_MIN, FOLLOW_BRAKE_SEAT_KP_SCALE_MAX, t)
        val brakeSeatKdScale = lerpClamped(FOLLOW_BRAKE_SEAT_KD_SCALE_MIN, FOLLOW_BRAKE_SEAT_KD_SCALE_MAX, t)
        val brakeTiltKpScale = lerpClamped(FOLLOW_BRAKE_TILT_KP_SCALE_MIN, FOLLOW_BRAKE_TILT_KP_SCALE_MAX, t)
        val brakeTiltKdScale = lerpClamped(FOLLOW_BRAKE_TILT_KD_SCALE_MIN, FOLLOW_BRAKE_TILT_KD_SCALE_MAX, t)
        val brakeTiltStiffnessFloor =
            lerpClamped(FOLLOW_BRAKE_TILT_STIFFNESS_FLOOR_MIN, FOLLOW_BRAKE_TILT_STIFFNESS_FLOOR_MAX, t)
                .coerceIn(0.0, 1.0)
        val brakeTiltDeadbandScale =
            lerpClamped(FOLLOW_BRAKE_TILT_DEADBAND_SCALE_MIN, FOLLOW_BRAKE_TILT_DEADBAND_SCALE_MAX, t)
                .coerceAtLeast(0.05)

        val chainBlend = smoothStep(FOLLOW_CHAIN_BLEND_START, FOLLOW_CHAIN_BLEND_END, t)
        val chainTorqueScale = lerpClamped(FOLLOW_CHAIN_TORQUE_SCALE_MIN, FOLLOW_CHAIN_TORQUE_SCALE_MAX, chainBlend)
            .coerceIn(0.0, 1.0)
        val chainStiffnessScale =
            lerpClamped(FOLLOW_CHAIN_STIFFNESS_SCALE_MIN, FOLLOW_CHAIN_STIFFNESS_SCALE_MAX, chainBlend)
                .coerceIn(0.0, 1.0)
        val holdWorldSeatKpBoost = lerpClamped(FOLLOW_HOLD_WORLD_SEAT_KP_BOOST_MIN, FOLLOW_HOLD_WORLD_SEAT_KP_BOOST_MAX, t)
        val holdWorldSeatKdBoost = lerpClamped(FOLLOW_HOLD_WORLD_SEAT_KD_BOOST_MIN, FOLLOW_HOLD_WORLD_SEAT_KD_BOOST_MAX, t)
        val holdWorldTiltKpBoost = lerpClamped(FOLLOW_HOLD_WORLD_TILT_KP_BOOST_MIN, FOLLOW_HOLD_WORLD_TILT_KP_BOOST_MAX, t)
        val holdWorldTiltKdBoost = lerpClamped(FOLLOW_HOLD_WORLD_TILT_KD_BOOST_MIN, FOLLOW_HOLD_WORLD_TILT_KD_BOOST_MAX, t)
        val holdWorldTiltAlphaCapScale =
            lerpClamped(FOLLOW_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MIN, FOLLOW_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MAX, t)
        val holdWorldTiltAlphaEqCapScale =
            lerpClamped(FOLLOW_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MIN, FOLLOW_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MAX, t)
        val holdRestTiltStiffnessFloor =
            lerpClamped(FOLLOW_HOLD_REST_TILT_STIFFNESS_FLOOR_MIN, FOLLOW_HOLD_REST_TILT_STIFFNESS_FLOOR_MAX, t)
                .coerceIn(0.0, 1.0)
        val verticalHoldSeatKpScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_SEAT_KP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_SEAT_KP_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldSeatKdScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_SEAT_KD_SCALE_MIN, FOLLOW_VERTICAL_HOLD_SEAT_KD_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldTiltKpScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_TILT_KP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_TILT_KP_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldTiltKdScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_TILT_KD_SCALE_MIN, FOLLOW_VERTICAL_HOLD_TILT_KD_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldExtraOffAxisDamping =
            lerpClamped(FOLLOW_VERTICAL_HOLD_EXTRA_OFFAXIS_DAMPING_MIN, FOLLOW_VERTICAL_HOLD_EXTRA_OFFAXIS_DAMPING_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldRestGateScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_REST_GATE_SCALE_MIN, FOLLOW_VERTICAL_HOLD_REST_GATE_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldWorldSeatAccelCapScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_WORLD_SEAT_ACCEL_CAP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_WORLD_SEAT_ACCEL_CAP_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldWorldTiltAlphaCapScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MAX, t)
                .coerceAtLeast(0.0)
        val verticalHoldWorldTiltAlphaEqCapScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MAX, t)
                .coerceAtLeast(0.0)

        return FollowStrengthParams(
            stopTimeSec = stopTimeSec,
            brakeRate = brakeRate,
            brakeStaticAlpha = brakeStaticAlpha,
            maxBrakeAlpha = maxBrakeAlpha,
            trackOmegaGain = trackOmegaGain,
            trackPosAssistGain = trackPosAssistGain,
            trackPosAssistOmegaLimit = trackPosAssistOmegaLimit,
            trackAuthorityFloor = trackAuthorityFloor,
            trackMaxOmegaStepPerTick = trackMaxOmegaStepPerTick,
            holdKpAlpha = holdKpAlpha,
            holdKdAlpha = holdKdAlpha,
            holdDampingZetaMin = holdDampingZetaMin,
            holdRingDownKdBoost = holdRingDownKdBoost,
            holdMaxAlpha = holdMaxAlpha,
            brakeSeatKpScale = brakeSeatKpScale,
            brakeSeatKdScale = brakeSeatKdScale,
            brakeTiltKpScale = brakeTiltKpScale,
            brakeTiltKdScale = brakeTiltKdScale,
            brakeTiltStiffnessFloor = brakeTiltStiffnessFloor,
            brakeTiltDeadbandScale = brakeTiltDeadbandScale,
            chainTorqueScale = chainTorqueScale,
            chainStiffnessScale = chainStiffnessScale,
            holdWorldSeatKpBoost = holdWorldSeatKpBoost,
            holdWorldSeatKdBoost = holdWorldSeatKdBoost,
            holdWorldTiltKpBoost = holdWorldTiltKpBoost,
            holdWorldTiltKdBoost = holdWorldTiltKdBoost,
            holdWorldTiltAlphaCapScale = holdWorldTiltAlphaCapScale,
            holdWorldTiltAlphaEqCapScale = holdWorldTiltAlphaEqCapScale,
            holdRestTiltStiffnessFloor = holdRestTiltStiffnessFloor,
            verticalHoldSeatKpScale = verticalHoldSeatKpScale,
            verticalHoldSeatKdScale = verticalHoldSeatKdScale,
            verticalHoldTiltKpScale = verticalHoldTiltKpScale,
            verticalHoldTiltKdScale = verticalHoldTiltKdScale,
            verticalHoldExtraOffAxisDamping = verticalHoldExtraOffAxisDamping,
            verticalHoldRestGateScale = verticalHoldRestGateScale,
            verticalHoldWorldSeatAccelCapScale = verticalHoldWorldSeatAccelCapScale,
            verticalHoldWorldTiltAlphaCapScale = verticalHoldWorldTiltAlphaCapScale,
            verticalHoldWorldTiltAlphaEqCapScale = verticalHoldWorldTiltAlphaEqCapScale
        )
    }

    fun sameDirectionAssistEnabled(errorRad: Double, commandedOmegaRadSec: Double): Boolean {
        if (!errorRad.isFinite() || !commandedOmegaRadSec.isFinite()) return false
        if (abs(errorRad) <= 1.0e-9 || abs(commandedOmegaRadSec) <= 1.0e-9) return false
        return errorRad * commandedOmegaRadSec > 0.0
    }

    fun isFollowCommandActive(commandAbsOmegaRadSec: Double, epsilonRadSec: Double): Boolean {
        if (!commandAbsOmegaRadSec.isFinite() || !epsilonRadSec.isFinite()) return false
        return commandAbsOmegaRadSec >= epsilonRadSec.coerceAtLeast(0.0)
    }

    fun verticalAxisBlend(absAxisY: Double, start: Double, end: Double): Double {
        if (!absAxisY.isFinite() || !start.isFinite() || !end.isFinite()) return 0.0
        return smoothStep(start, end, absAxisY.coerceIn(0.0, 1.0))
    }

    fun computeAxisHoldProfile(absAxisY: Double, strength01: Double): AxisHoldProfile {
        if (!absAxisY.isFinite() || !strength01.isFinite()) return AxisHoldProfile.IDENTITY

        val t = strength01.coerceIn(0.0, 1.0)
        val y = absAxisY.coerceIn(0.0, 1.0)
        val verticalBlend = verticalAxisBlend(y, FOLLOW_AXIS_BLEND_START, FOLLOW_AXIS_BLEND_END)
        val horizontalBlend = verticalAxisBlend(1.0 - y, FOLLOW_AXIS_BLEND_START, FOLLOW_AXIS_BLEND_END)

        val verticalSeatKpScale = lerpClamped(FOLLOW_VERTICAL_HOLD_SEAT_KP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_SEAT_KP_SCALE_MAX, t)
        val verticalSeatKdScale = lerpClamped(FOLLOW_VERTICAL_HOLD_SEAT_KD_SCALE_MIN, FOLLOW_VERTICAL_HOLD_SEAT_KD_SCALE_MAX, t)
        val verticalTiltKpScale = lerpClamped(FOLLOW_VERTICAL_HOLD_TILT_KP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_TILT_KP_SCALE_MAX, t)
        val verticalTiltKdScale = lerpClamped(FOLLOW_VERTICAL_HOLD_TILT_KD_SCALE_MIN, FOLLOW_VERTICAL_HOLD_TILT_KD_SCALE_MAX, t)
        val verticalRestGateScale = lerpClamped(FOLLOW_VERTICAL_HOLD_REST_GATE_SCALE_MIN, FOLLOW_VERTICAL_HOLD_REST_GATE_SCALE_MAX, t)
        val verticalExtraOffAxisDamping =
            lerpClamped(FOLLOW_VERTICAL_HOLD_EXTRA_OFFAXIS_DAMPING_MIN, FOLLOW_VERTICAL_HOLD_EXTRA_OFFAXIS_DAMPING_MAX, t)
        val verticalWorldSeatAccelCapScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_WORLD_SEAT_ACCEL_CAP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_WORLD_SEAT_ACCEL_CAP_SCALE_MAX, t)
        val verticalWorldTiltAlphaCapScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MAX, t)
        val verticalWorldTiltAlphaEqCapScale =
            lerpClamped(FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MIN, FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MAX, t)

        val horizontalSeatKpScale =
            lerpClamped(FOLLOW_HORIZONTAL_HOLD_SEAT_KP_SCALE_MIN, FOLLOW_HORIZONTAL_HOLD_SEAT_KP_SCALE_MAX, t)
        val horizontalSeatKdScale =
            lerpClamped(FOLLOW_HORIZONTAL_HOLD_SEAT_KD_SCALE_MIN, FOLLOW_HORIZONTAL_HOLD_SEAT_KD_SCALE_MAX, t)
        val horizontalTiltKpScale =
            lerpClamped(FOLLOW_HORIZONTAL_HOLD_TILT_KP_SCALE_MIN, FOLLOW_HORIZONTAL_HOLD_TILT_KP_SCALE_MAX, t)
        val horizontalTiltKdScale =
            lerpClamped(FOLLOW_HORIZONTAL_HOLD_TILT_KD_SCALE_MIN, FOLLOW_HORIZONTAL_HOLD_TILT_KD_SCALE_MAX, t)
        val horizontalRestGateScale =
            lerpClamped(FOLLOW_HORIZONTAL_HOLD_REST_GATE_SCALE_MIN, FOLLOW_HORIZONTAL_HOLD_REST_GATE_SCALE_MAX, t)
        val horizontalExtraOffAxisDamping =
            lerpClamped(FOLLOW_HORIZONTAL_HOLD_EXTRA_OFFAXIS_DAMPING_MIN, FOLLOW_HORIZONTAL_HOLD_EXTRA_OFFAXIS_DAMPING_MAX, t)

        val seatKpScale =
            lerpClamped(1.0, verticalSeatKpScale, verticalBlend) * lerpClamped(1.0, horizontalSeatKpScale, horizontalBlend)
        val seatKdScale =
            lerpClamped(1.0, verticalSeatKdScale, verticalBlend) * lerpClamped(1.0, horizontalSeatKdScale, horizontalBlend)
        val tiltKpScale =
            lerpClamped(1.0, verticalTiltKpScale, verticalBlend) * lerpClamped(1.0, horizontalTiltKpScale, horizontalBlend)
        val tiltKdScale =
            lerpClamped(1.0, verticalTiltKdScale, verticalBlend) * lerpClamped(1.0, horizontalTiltKdScale, horizontalBlend)
        val restGateScale =
            (lerpClamped(1.0, verticalRestGateScale, verticalBlend) * lerpClamped(1.0, horizontalRestGateScale, horizontalBlend))
                .coerceIn(0.05, 1.0)
        val extraOffAxisDamping =
            (verticalExtraOffAxisDamping * verticalBlend + horizontalExtraOffAxisDamping * horizontalBlend).coerceAtLeast(0.0)

        val verticalMicroKpFloor = lerpClamped(FOLLOW_VERTICAL_MICRO_KP_FLOOR_MIN, FOLLOW_VERTICAL_MICRO_KP_FLOOR_MAX, t)
        val horizontalMicroKpFloor = lerpClamped(FOLLOW_HORIZONTAL_MICRO_KP_FLOOR_MIN, FOLLOW_HORIZONTAL_MICRO_KP_FLOOR_MAX, t)
        val verticalMicroKdBoost = lerpClamped(FOLLOW_VERTICAL_MICRO_KD_BOOST_MIN, FOLLOW_VERTICAL_MICRO_KD_BOOST_MAX, t)
        val horizontalMicroKdBoost = lerpClamped(FOLLOW_HORIZONTAL_MICRO_KD_BOOST_MIN, FOLLOW_HORIZONTAL_MICRO_KD_BOOST_MAX, t)
        val microKpFloor =
            (lerpClamped(1.0, verticalMicroKpFloor, verticalBlend) * lerpClamped(1.0, horizontalMicroKpFloor, horizontalBlend))
                .coerceIn(0.05, 1.0)
        val microKdBoost =
            (lerpClamped(1.0, verticalMicroKdBoost, verticalBlend) * lerpClamped(1.0, horizontalMicroKdBoost, horizontalBlend))
                .coerceAtLeast(1.0)

        val horizontalZetaMin = lerpClamped(FOLLOW_HORIZONTAL_OFFAXIS_ZETA_MIN_MIN, FOLLOW_HORIZONTAL_OFFAXIS_ZETA_MIN_MAX, t)
        val verticalZetaMin = lerpClamped(FOLLOW_VERTICAL_OFFAXIS_ZETA_MIN_MIN, FOLLOW_VERTICAL_OFFAXIS_ZETA_MIN_MAX, t)
        val offAxisDampingZetaMin =
            lerpClamped(horizontalZetaMin, verticalZetaMin, verticalBlend).coerceAtLeast(1.0)

        return AxisHoldProfile(
            verticalBlend = verticalBlend,
            horizontalBlend = horizontalBlend,
            seatKpScale = seatKpScale.coerceAtLeast(0.0),
            seatKdScale = seatKdScale.coerceAtLeast(0.0),
            tiltKpScale = tiltKpScale.coerceAtLeast(0.0),
            tiltKdScale = tiltKdScale.coerceAtLeast(0.0),
            restGateScale = restGateScale,
            extraOffAxisDamping = extraOffAxisDamping,
            worldSeatAccelCapScale = lerpClamped(1.0, verticalWorldSeatAccelCapScale, verticalBlend).coerceAtLeast(1.0),
            worldTiltAlphaCapScale = lerpClamped(1.0, verticalWorldTiltAlphaCapScale, verticalBlend).coerceAtLeast(1.0),
            worldTiltAlphaEqCapScale = lerpClamped(1.0, verticalWorldTiltAlphaEqCapScale, verticalBlend).coerceAtLeast(1.0),
            microKpFloor = microKpFloor,
            microKdBoost = microKdBoost,
            offAxisDampingZetaMin = offAxisDampingZetaMin
        )
    }

    fun boundedStallEpsilonRad(
        cmdDeltaRad: Double,
        baseEpsRad: Double,
        commandFraction: Double,
        maxEpsRad: Double
    ): Double {
        if (!cmdDeltaRad.isFinite() || !baseEpsRad.isFinite() || !commandFraction.isFinite() || !maxEpsRad.isFinite()) return 0.0
        val base = baseEpsRad.coerceAtLeast(0.0)
        val maxEps = maxEpsRad.coerceAtLeast(base)
        val cmdTerm = abs(cmdDeltaRad) * commandFraction.coerceAtLeast(0.0)
        return (base + cmdTerm).coerceAtMost(maxEps)
    }

    fun isSignificantCommandStep(
        previousMagnitude: Double,
        currentMagnitude: Double,
        previousSign: Int,
        currentSign: Int,
        absoluteStepThreshold: Double,
        relativeStepThreshold: Double
    ): Boolean {
        if (!previousMagnitude.isFinite() || !currentMagnitude.isFinite()) return true
        if (previousSign != 0 && currentSign != 0 && previousSign != currentSign) return true
        val prev = abs(previousMagnitude)
        val curr = abs(currentMagnitude)
        val absDelta = abs(curr - prev)
        val absStep = absDelta >= absoluteStepThreshold.coerceAtLeast(0.0)
        val relBase = max(prev, 1.0e-6)
        val relStep = (absDelta / relBase) >= relativeStepThreshold.coerceAtLeast(0.0)
        return absStep || relStep
    }

    fun stepFollowStallState(
        previous: FollowStallState,
        stallCountEligible: Boolean,
        clearMotionEligible: Boolean,
        commandStep: Boolean,
        stallTicksRequired: Int,
        clearTicksRequired: Int,
        graceTicksOnCommandStep: Int
    ): FollowStallState {
        val stallRequired = stallTicksRequired.coerceAtLeast(1)
        val clearRequired = clearTicksRequired.coerceAtLeast(1)
        val graceTicks = if (commandStep) {
            graceTicksOnCommandStep.coerceAtLeast(0)
        } else {
            max(0, previous.graceTicks - 1)
        }

        if (previous.stalled) {
            if (commandStep) {
                return FollowStallState(stalled = false, stallTicks = 0, clearTicks = 0, graceTicks = graceTicks)
            }
            val clearTicks = if (clearMotionEligible) previous.clearTicks + 1 else 0
            if (clearTicks >= clearRequired) {
                return FollowStallState(stalled = false, stallTicks = 0, clearTicks = 0, graceTicks = graceTicks)
            }
            return FollowStallState(stalled = true, stallTicks = 0, clearTicks = clearTicks, graceTicks = graceTicks)
        }

        val canCount = graceTicks <= 0 && stallCountEligible
        val stallTicks = if (canCount) previous.stallTicks + 1 else 0
        val stalled = stallTicks >= stallRequired
        return if (stalled) {
            FollowStallState(stalled = true, stallTicks = 0, clearTicks = 0, graceTicks = graceTicks)
        } else {
            FollowStallState(stalled = false, stallTicks = stallTicks, clearTicks = 0, graceTicks = graceTicks)
        }
    }

    fun computeTrackAuthority(
        chainFactor: Double,
        offAxisFactor: Double,
        relVelFactor: Double,
        minAuthority: Double
    ): Double {
        if (!chainFactor.isFinite() || !offAxisFactor.isFinite() || !relVelFactor.isFinite() || !minAuthority.isFinite()) return 0.0
        val minA = minAuthority.coerceIn(0.0, 1.0)
        return (chainFactor * offAxisFactor * relVelFactor).coerceIn(minA, 1.0)
    }

    fun computeTrackOmegaTarget(
        commandOmegaRadSec: Double,
        errorRad: Double,
        posAssistGain: Double,
        posAssistOmegaLimit: Double,
        maxOmegaRadSec: Double
    ): Double {
        if (
            !commandOmegaRadSec.isFinite() ||
            !errorRad.isFinite() ||
            !posAssistGain.isFinite() ||
            !posAssistOmegaLimit.isFinite() ||
            !maxOmegaRadSec.isFinite()
        ) {
            return 0.0
        }
        val posAssistAllowed = sameDirectionAssistEnabled(errorRad, commandOmegaRadSec)
        val omegaFromPosition = if (posAssistAllowed) {
            (posAssistGain * errorRad).coerceIn(-posAssistOmegaLimit, posAssistOmegaLimit)
        } else {
            0.0
        }
        return (commandOmegaRadSec + omegaFromPosition).coerceIn(-maxOmegaRadSec, maxOmegaRadSec)
    }

    fun computeFrictionBrakeAlpha(
        omegaActualRadSec: Double,
        brakeRate: Double,
        staticBrakeAlpha: Double,
        stopOmegaRadSec: Double
    ): Double {
        if (!omegaActualRadSec.isFinite() || !brakeRate.isFinite() || !staticBrakeAlpha.isFinite()) return 0.0
        if (abs(omegaActualRadSec) <= stopOmegaRadSec.coerceAtLeast(0.0)) return 0.0
        val viscous = -brakeRate * omegaActualRadSec
        val coulomb = -sign(omegaActualRadSec) * staticBrakeAlpha
        return viscous + coulomb
    }

    fun computeSnapToZeroAlpha(omegaActualRadSec: Double, dtSec: Double): Double {
        if (!omegaActualRadSec.isFinite() || !dtSec.isFinite() || dtSec <= 1.0e-9) return 0.0
        return -omegaActualRadSec / dtSec
    }

    fun holdKdWithDampingFloor(
        holdKpAlpha: Double,
        holdKdAlphaMapped: Double,
        holdDampingZetaMin: Double
    ): Double {
        return kdWithFloor(holdKpAlpha, holdKdAlphaMapped, holdDampingZetaMin)
    }

    fun kdWithFloor(kp: Double, kdMapped: Double, zetaMin: Double): Double {
        if (!kp.isFinite() || !kdMapped.isFinite() || !zetaMin.isFinite()) return 0.0
        val kpSafe = max(kp, 1.0e-9)
        val kdFloor = 2.0 * zetaMin.coerceAtLeast(0.0) * sqrt(kpSafe)
        return max(kdMapped, kdFloor)
    }

    fun computeHoldAlpha(
        holdErrorRad: Double,
        omegaActualRadSec: Double,
        holdKpAlpha: Double,
        holdKdAlphaMapped: Double,
        holdDampingZetaMin: Double,
        holdErrorDeadbandRad: Double,
        holdOmegaDeadbandRadSec: Double
    ): Double {
        if (
            !holdErrorRad.isFinite() ||
            !omegaActualRadSec.isFinite() ||
            !holdKpAlpha.isFinite() ||
            !holdKdAlphaMapped.isFinite() ||
            !holdDampingZetaMin.isFinite()
        ) {
            return 0.0
        }
        val errorDeadband = holdErrorDeadbandRad.coerceAtLeast(0.0)
        val omegaDeadband = holdOmegaDeadbandRadSec.coerceAtLeast(0.0)
        if (abs(holdErrorRad) <= errorDeadband && abs(omegaActualRadSec) <= omegaDeadband) return 0.0

        val kdEff = holdKdWithDampingFloor(holdKpAlpha, holdKdAlphaMapped, holdDampingZetaMin)
        return holdKpAlpha * holdErrorRad - kdEff * omegaActualRadSec
    }

    fun updateHoldRingDown(
        currentTicks: Int,
        previousErrorSign: Int,
        holdErrorRad: Double,
        omegaActualRadSec: Double,
        errorMinRad: Double,
        omegaMinRadSec: Double,
        durationTicks: Int
    ): RingDownState {
        val errorMin = errorMinRad.coerceAtLeast(0.0)
        val omegaMin = omegaMinRadSec.coerceAtLeast(0.0)
        val significant = abs(holdErrorRad) >= errorMin && abs(omegaActualRadSec) >= omegaMin
        val signNow = if (significant) sign(holdErrorRad).toInt() else 0
        val flipDetected = signNow != 0 && previousErrorSign != 0 && signNow != previousErrorSign
        val ticks = when {
            flipDetected -> durationTicks.coerceAtLeast(0)
            currentTicks > 0 -> currentTicks - 1
            else -> 0
        }
        val errorSign = when {
            signNow != 0 -> signNow
            abs(holdErrorRad) <= errorMin * 0.5 && abs(omegaActualRadSec) <= omegaMin * 0.5 -> 0
            else -> previousErrorSign
        }
        return RingDownState(ticks = ticks, errorSign = errorSign)
    }

    fun holdRingDownKdMultiplier(ringDownTicks: Int, configuredBoost: Double): Double {
        if (ringDownTicks <= 0) return 1.0
        return configuredBoost.coerceAtLeast(1.0)
    }

    fun stepHysteresisLatch(
        previouslyActive: Boolean,
        previousTicks: Int,
        eligible: Boolean,
        enterCondition: Boolean,
        exitCondition: Boolean,
        enterTicksRequired: Int
    ): HysteresisLatchState {
        if (!eligible) return HysteresisLatchState(active = false, ticks = 0)
        if (previouslyActive) {
            return if (exitCondition) {
                HysteresisLatchState(active = false, ticks = 0)
            } else {
                HysteresisLatchState(active = true, ticks = max(previousTicks, enterTicksRequired))
            }
        }
        val ticks = if (enterCondition) previousTicks + 1 else 0
        val active = enterTicksRequired <= 0 || ticks >= enterTicksRequired
        return HysteresisLatchState(active = active, ticks = ticks)
    }

    fun holdRestAuthorityFloor(strength01: Double, rigidBlend01: Double): Double {
        val t = strength01.coerceIn(0.0, 1.0)
        val base = lerpClamped(FOLLOW_HOLD_AUTH_FLOOR_MIN, FOLLOW_HOLD_AUTH_FLOOR_MAX, t)
        val rigidBoost = FOLLOW_HOLD_AUTH_FLOOR_RIGID_BOOST * rigidBlend01.coerceIn(0.0, 1.0)
        return (base + rigidBoost).coerceIn(0.0, 1.0)
    }

    fun holdOffAxisDampingBoost(strength01: Double, rigidBlend01: Double): Double {
        val t = strength01.coerceIn(0.0, 1.0)
        val base = lerpClamped(FOLLOW_OFFAXIS_DAMP_BOOST_MIN, FOLLOW_OFFAXIS_DAMP_BOOST_MAX, t)
        val rigidBoost = FOLLOW_OFFAXIS_DAMP_BOOST_RIGID * rigidBlend01.coerceIn(0.0, 1.0)
        return (base + rigidBoost).coerceAtLeast(1.0)
    }

    fun stepFollowPhase(
        trackActive: Boolean,
        omegaAbsRadSec: Double,
        previous: FollowPhaseState,
        captureOmegaRadSec: Double,
        captureTicksRequired: Int,
        emergencyReleaseOmegaRadSec: Double,
        emergencyReleaseTicksRequired: Int
    ): FollowPhaseState {
        if (trackActive) {
            return FollowPhaseState(
                phase = FollowPhase.TRACK,
                holdLatched = false,
                captureTicks = 0,
                emergencyTicks = 0
            )
        }

        if (previous.holdLatched) {
            val emergencyTicks = if (omegaAbsRadSec >= emergencyReleaseOmegaRadSec.coerceAtLeast(0.0)) {
                previous.emergencyTicks + 1
            } else {
                0
            }
            val release = emergencyReleaseTicksRequired > 0 && emergencyTicks >= emergencyReleaseTicksRequired
            return if (release) {
                FollowPhaseState(
                    phase = FollowPhase.BRAKE_DECEL,
                    holdLatched = false,
                    captureTicks = 0,
                    emergencyTicks = 0
                )
            } else {
                FollowPhaseState(
                    phase = FollowPhase.BRAKE_HOLD,
                    holdLatched = true,
                    captureTicks = previous.captureTicks,
                    emergencyTicks = emergencyTicks
                )
            }
        }

        val captureTicks = if (omegaAbsRadSec <= captureOmegaRadSec.coerceAtLeast(0.0)) {
            previous.captureTicks + 1
        } else {
            0
        }
        val captured = captureTicksRequired <= 0 || captureTicks >= captureTicksRequired
        return if (captured) {
            FollowPhaseState(
                phase = FollowPhase.BRAKE_HOLD,
                holdLatched = true,
                captureTicks = captureTicks,
                emergencyTicks = 0
            )
        } else {
            FollowPhaseState(
                phase = FollowPhase.BRAKE_DECEL,
                holdLatched = false,
                captureTicks = captureTicks,
                emergencyTicks = 0
            )
        }
    }

    fun clampNoReverse(alphaCmdRadSec2: Double, omegaActualRadSec: Double, dtSec: Double): Double {
        if (!alphaCmdRadSec2.isFinite() || !omegaActualRadSec.isFinite()) return 0.0
        if (dtSec <= 1.0e-9) return alphaCmdRadSec2
        if (abs(omegaActualRadSec) <= 1.0e-9) return alphaCmdRadSec2

        val maxOpposingAlpha = abs(omegaActualRadSec) / dtSec
        return if (omegaActualRadSec > 0.0) {
            alphaCmdRadSec2.coerceAtLeast(-maxOpposingAlpha)
        } else {
            alphaCmdRadSec2.coerceAtMost(maxOpposingAlpha)
        }
    }

    private fun lerpLog(min: Double, max: Double, t: Double): Double {
        if (min <= 0.0 || max <= 0.0) return min + (max - min) * t
        val lnMin = ln(min)
        val lnMax = ln(max)
        return exp(lnMin + (lnMax - lnMin) * t)
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

    private const val FOLLOW_STOP_TIME_MIN_SEC = 0.08
    private const val FOLLOW_STOP_TIME_MAX_SEC = 0.60
    private const val FOLLOW_STOP_SETTLE_RATIO = 100.0
    private const val FOLLOW_STATIC_BRAKE_ALPHA_MIN = 6.0
    private const val FOLLOW_STATIC_BRAKE_ALPHA_MAX = 96.0
    private const val FOLLOW_MAX_BRAKE_ALPHA_MIN = 26.0
    private const val FOLLOW_MAX_BRAKE_ALPHA_MAX = 180.0
    private const val FOLLOW_TRACK_OMEGA_GAIN_MIN = 2.4
    private const val FOLLOW_TRACK_OMEGA_GAIN_MAX = 18.0
    private const val FOLLOW_TRACK_POS_GAIN_MIN = 1.2
    private const val FOLLOW_TRACK_POS_GAIN_MAX = 12.0
    private const val FOLLOW_TRACK_POS_OMEGA_LIMIT_MIN = 2.0
    private const val FOLLOW_TRACK_POS_OMEGA_LIMIT_MAX = 22.0
    private const val FOLLOW_TRACK_AUTH_FLOOR_MIN = 0.22
    private const val FOLLOW_TRACK_AUTH_FLOOR_MAX = 0.62
    private const val FOLLOW_TRACK_MAX_OMEGA_STEP_PER_TICK_MIN = 1.4
    private const val FOLLOW_TRACK_MAX_OMEGA_STEP_PER_TICK_MAX = 3.2
    private const val FOLLOW_HOLD_KP_ALPHA_MIN = 10.0
    private const val FOLLOW_HOLD_KP_ALPHA_MAX = 90.0
    private const val FOLLOW_HOLD_KD_ALPHA_MIN = 2.0
    private const val FOLLOW_HOLD_KD_ALPHA_MAX = 16.0
    private const val FOLLOW_HOLD_DAMPING_ZETA_MIN = 1.1
    private const val FOLLOW_HOLD_DAMPING_ZETA_MAX = 1.5
    private const val FOLLOW_HOLD_RINGDOWN_KD_BOOST_MIN = 1.25
    private const val FOLLOW_HOLD_RINGDOWN_KD_BOOST_MAX = 1.80
    private const val FOLLOW_HOLD_MAX_ALPHA_MIN = 8.0
    private const val FOLLOW_HOLD_MAX_ALPHA_MAX = 120.0
    private const val FOLLOW_BRAKE_SEAT_KP_SCALE_MIN = 0.45
    private const val FOLLOW_BRAKE_SEAT_KP_SCALE_MAX = 0.90
    private const val FOLLOW_BRAKE_SEAT_KD_SCALE_MIN = 1.30
    private const val FOLLOW_BRAKE_SEAT_KD_SCALE_MAX = 2.20
    private const val FOLLOW_BRAKE_TILT_KP_SCALE_MIN = 0.35
    private const val FOLLOW_BRAKE_TILT_KP_SCALE_MAX = 0.85
    private const val FOLLOW_BRAKE_TILT_KD_SCALE_MIN = 1.35
    private const val FOLLOW_BRAKE_TILT_KD_SCALE_MAX = 2.25
    private const val FOLLOW_BRAKE_TILT_STIFFNESS_FLOOR_MIN = 0.16
    private const val FOLLOW_BRAKE_TILT_STIFFNESS_FLOOR_MAX = 0.40
    private const val FOLLOW_BRAKE_TILT_DEADBAND_SCALE_MIN = 0.80
    private const val FOLLOW_BRAKE_TILT_DEADBAND_SCALE_MAX = 0.45
    private const val FOLLOW_CHAIN_BLEND_START = 0.20
    private const val FOLLOW_CHAIN_BLEND_END = 0.70
    private const val FOLLOW_CHAIN_TORQUE_SCALE_MIN = 0.45
    private const val FOLLOW_CHAIN_TORQUE_SCALE_MAX = 0.85
    private const val FOLLOW_CHAIN_STIFFNESS_SCALE_MIN = 0.35
    private const val FOLLOW_CHAIN_STIFFNESS_SCALE_MAX = 0.75
    private const val FOLLOW_HOLD_WORLD_SEAT_KP_BOOST_MIN = 1.05
    private const val FOLLOW_HOLD_WORLD_SEAT_KP_BOOST_MAX = 1.45
    private const val FOLLOW_HOLD_WORLD_SEAT_KD_BOOST_MIN = 1.10
    private const val FOLLOW_HOLD_WORLD_SEAT_KD_BOOST_MAX = 1.65
    private const val FOLLOW_HOLD_WORLD_TILT_KP_BOOST_MIN = 1.05
    private const val FOLLOW_HOLD_WORLD_TILT_KP_BOOST_MAX = 1.40
    private const val FOLLOW_HOLD_WORLD_TILT_KD_BOOST_MIN = 1.12
    private const val FOLLOW_HOLD_WORLD_TILT_KD_BOOST_MAX = 1.75
    private const val FOLLOW_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MIN = 1.05
    private const val FOLLOW_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MAX = 1.45
    private const val FOLLOW_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MIN = 1.05
    private const val FOLLOW_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MAX = 1.38
    private const val FOLLOW_HOLD_REST_TILT_STIFFNESS_FLOOR_MIN = 0.32
    private const val FOLLOW_HOLD_REST_TILT_STIFFNESS_FLOOR_MAX = 0.64
    private const val FOLLOW_VERTICAL_HOLD_SEAT_KP_SCALE_MIN = 1.00
    private const val FOLLOW_VERTICAL_HOLD_SEAT_KP_SCALE_MAX = 0.82
    private const val FOLLOW_VERTICAL_HOLD_SEAT_KD_SCALE_MIN = 1.10
    private const val FOLLOW_VERTICAL_HOLD_SEAT_KD_SCALE_MAX = 2.20
    private const val FOLLOW_VERTICAL_HOLD_TILT_KP_SCALE_MIN = 1.00
    private const val FOLLOW_VERTICAL_HOLD_TILT_KP_SCALE_MAX = 0.78
    private const val FOLLOW_VERTICAL_HOLD_TILT_KD_SCALE_MIN = 1.15
    private const val FOLLOW_VERTICAL_HOLD_TILT_KD_SCALE_MAX = 2.35
    private const val FOLLOW_VERTICAL_HOLD_EXTRA_OFFAXIS_DAMPING_MIN = 0.20
    private const val FOLLOW_VERTICAL_HOLD_EXTRA_OFFAXIS_DAMPING_MAX = 2.00
    private const val FOLLOW_VERTICAL_HOLD_REST_GATE_SCALE_MIN = 1.00
    private const val FOLLOW_VERTICAL_HOLD_REST_GATE_SCALE_MAX = 0.30
    private const val FOLLOW_VERTICAL_HOLD_WORLD_SEAT_ACCEL_CAP_SCALE_MIN = 1.00
    private const val FOLLOW_VERTICAL_HOLD_WORLD_SEAT_ACCEL_CAP_SCALE_MAX = 2.10
    private const val FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MIN = 1.00
    private const val FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_CAP_SCALE_MAX = 2.20
    private const val FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MIN = 1.00
    private const val FOLLOW_VERTICAL_HOLD_WORLD_TILT_ALPHA_EQ_CAP_SCALE_MAX = 2.00
    private const val FOLLOW_HORIZONTAL_HOLD_SEAT_KP_SCALE_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_HOLD_SEAT_KP_SCALE_MAX = 0.92
    private const val FOLLOW_HORIZONTAL_HOLD_SEAT_KD_SCALE_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_HOLD_SEAT_KD_SCALE_MAX = 1.45
    private const val FOLLOW_HORIZONTAL_HOLD_TILT_KP_SCALE_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_HOLD_TILT_KP_SCALE_MAX = 0.90
    private const val FOLLOW_HORIZONTAL_HOLD_TILT_KD_SCALE_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_HOLD_TILT_KD_SCALE_MAX = 1.55
    private const val FOLLOW_HORIZONTAL_HOLD_REST_GATE_SCALE_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_HOLD_REST_GATE_SCALE_MAX = 0.65
    private const val FOLLOW_HORIZONTAL_HOLD_EXTRA_OFFAXIS_DAMPING_MIN = 0.00
    private const val FOLLOW_HORIZONTAL_HOLD_EXTRA_OFFAXIS_DAMPING_MAX = 0.70
    private const val FOLLOW_AXIS_BLEND_START = 0.55
    private const val FOLLOW_AXIS_BLEND_END = 0.90
    private const val FOLLOW_VERTICAL_MICRO_KP_FLOOR_MIN = 1.00
    private const val FOLLOW_VERTICAL_MICRO_KP_FLOOR_MAX = 0.45
    private const val FOLLOW_VERTICAL_MICRO_KD_BOOST_MIN = 1.00
    private const val FOLLOW_VERTICAL_MICRO_KD_BOOST_MAX = 1.50
    private const val FOLLOW_HORIZONTAL_MICRO_KP_FLOOR_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_MICRO_KP_FLOOR_MAX = 0.72
    private const val FOLLOW_HORIZONTAL_MICRO_KD_BOOST_MIN = 1.00
    private const val FOLLOW_HORIZONTAL_MICRO_KD_BOOST_MAX = 1.20
    private const val FOLLOW_HORIZONTAL_OFFAXIS_ZETA_MIN_MIN = 1.10
    private const val FOLLOW_HORIZONTAL_OFFAXIS_ZETA_MIN_MAX = 1.35
    private const val FOLLOW_VERTICAL_OFFAXIS_ZETA_MIN_MIN = 1.20
    private const val FOLLOW_VERTICAL_OFFAXIS_ZETA_MIN_MAX = 1.55
    private const val FOLLOW_HOLD_AUTH_FLOOR_MIN = 0.48
    private const val FOLLOW_HOLD_AUTH_FLOOR_MAX = 0.78
    private const val FOLLOW_HOLD_AUTH_FLOOR_RIGID_BOOST = 0.15
    private const val FOLLOW_OFFAXIS_DAMP_BOOST_MIN = 1.08
    private const val FOLLOW_OFFAXIS_DAMP_BOOST_MAX = 1.55
    private const val FOLLOW_OFFAXIS_DAMP_BOOST_RIGID = 0.20
}
