package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.sign

internal object PhysBearingServoMath {
    data class FollowStrengthParams(
        val stopTimeSec: Double,
        val brakeRate: Double,
        val brakeStaticAlpha: Double,
        val maxBrakeAlpha: Double,
        val trackOmegaGain: Double,
        val trackPosAssistGain: Double,
        val trackPosAssistOmegaLimit: Double,
        val holdPosGain: Double,
        val holdOmegaGain: Double,
        val holdPosOmegaLimit: Double,
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
        val holdPosGain = lerpLog(FOLLOW_HOLD_POS_GAIN_MIN, FOLLOW_HOLD_POS_GAIN_MAX, t) * scale
        val holdOmegaGain = lerpLog(FOLLOW_HOLD_OMEGA_GAIN_MIN, FOLLOW_HOLD_OMEGA_GAIN_MAX, t) * scale
        val holdPosOmegaLimit = lerpLog(FOLLOW_HOLD_POS_OMEGA_LIMIT_MIN, FOLLOW_HOLD_POS_OMEGA_LIMIT_MAX, t) * scale
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
            holdPosGain = holdPosGain,
            holdOmegaGain = holdOmegaGain,
            holdPosOmegaLimit = holdPosOmegaLimit,
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

    fun computeHoldAlpha(
        holdErrorRad: Double,
        omegaActualRadSec: Double,
        holdPosGain: Double,
        holdOmegaGain: Double,
        holdPosOmegaLimit: Double,
        holdErrorDeadbandRad: Double,
        holdOmegaDeadbandRadSec: Double
    ): Double {
        if (
            !holdErrorRad.isFinite() ||
            !omegaActualRadSec.isFinite() ||
            !holdPosGain.isFinite() ||
            !holdOmegaGain.isFinite() ||
            !holdPosOmegaLimit.isFinite()
        ) {
            return 0.0
        }
        val errorDeadband = holdErrorDeadbandRad.coerceAtLeast(0.0)
        val omegaDeadband = holdOmegaDeadbandRadSec.coerceAtLeast(0.0)
        if (abs(holdErrorRad) <= errorDeadband && abs(omegaActualRadSec) <= omegaDeadband) return 0.0

        val omegaLimit = abs(holdPosOmegaLimit)
        val omegaTarget = (holdPosGain * holdErrorRad).coerceIn(-omegaLimit, omegaLimit)
        val omegaErr = omegaTarget - omegaActualRadSec
        return holdOmegaGain * omegaErr
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
    private const val FOLLOW_HOLD_POS_GAIN_MIN = 2.2
    private const val FOLLOW_HOLD_POS_GAIN_MAX = 12.0
    private const val FOLLOW_HOLD_OMEGA_GAIN_MIN = 3.0
    private const val FOLLOW_HOLD_OMEGA_GAIN_MAX = 22.0
    private const val FOLLOW_HOLD_POS_OMEGA_LIMIT_MIN = 0.35
    private const val FOLLOW_HOLD_POS_OMEGA_LIMIT_MAX = 3.8
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
}
