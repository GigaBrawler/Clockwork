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

    data class FollowStrengthParams(
        val stopTimeSec: Double,
        val brakeRate: Double,
        val brakeStaticAlpha: Double,
        val maxBrakeAlpha: Double,
        val trackOmegaGain: Double,
        val trackPosAssistGain: Double,
        val trackPosAssistOmegaLimit: Double,
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

        return FollowStrengthParams(
            stopTimeSec = stopTimeSec,
            brakeRate = brakeRate,
            brakeStaticAlpha = brakeStaticAlpha,
            maxBrakeAlpha = maxBrakeAlpha,
            trackOmegaGain = trackOmegaGain,
            trackPosAssistGain = trackPosAssistGain,
            trackPosAssistOmegaLimit = trackPosAssistOmegaLimit,
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
            chainStiffnessScale = chainStiffnessScale
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
        if (!holdKpAlpha.isFinite() || !holdKdAlphaMapped.isFinite() || !holdDampingZetaMin.isFinite()) return 0.0
        val kp = max(holdKpAlpha, 1.0e-9)
        val kdFloor = 2.0 * holdDampingZetaMin.coerceAtLeast(0.0) * sqrt(kp)
        return max(holdKdAlphaMapped, kdFloor)
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
    private const val FOLLOW_HOLD_AUTH_FLOOR_MIN = 0.48
    private const val FOLLOW_HOLD_AUTH_FLOOR_MAX = 0.78
    private const val FOLLOW_HOLD_AUTH_FLOOR_RIGID_BOOST = 0.15
    private const val FOLLOW_OFFAXIS_DAMP_BOOST_MIN = 1.08
    private const val FOLLOW_OFFAXIS_DAMP_BOOST_MAX = 1.55
    private const val FOLLOW_OFFAXIS_DAMP_BOOST_RIGID = 0.20
}
