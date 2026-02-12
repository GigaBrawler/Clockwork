package org.valkyrienskies.clockwork.util

import net.minecraft.server.level.ServerLevel
import org.valkyrienskies.core.api.world.properties.DimensionId
import org.valkyrienskies.mod.common.IShipObjectWorldServerProvider

private const val FALLBACK_AIR_PRESSURE = 101325.0
private const val FALLBACK_AIR_DENSITY = 1.225
private const val FALLBACK_AIR_TEMPERATURE = 288.15
private const val MIN_POSITIVE = 1e-6

fun safeAirPressure(level: ServerLevel?, y: Double, dimensionId: DimensionId): Double {
    val value = runCatching {
        (level as? IShipObjectWorldServerProvider)
            ?.shipObjectWorld
            ?.aerodynamicUtils
            ?.getAirPressureForY(y, dimensionId)
    }.getOrNull()
    return sanitizePositive(value, FALLBACK_AIR_PRESSURE)
}

fun safeAirDensity(level: ServerLevel?, y: Double, dimensionId: DimensionId): Double {
    val value = runCatching {
        (level as? IShipObjectWorldServerProvider)
            ?.shipObjectWorld
            ?.aerodynamicUtils
            ?.getAirDensityForY(y, dimensionId)
    }.getOrNull()
    return sanitizePositive(value, FALLBACK_AIR_DENSITY)
}

fun safeAirTemperature(level: ServerLevel?, y: Double, dimensionId: DimensionId): Double {
    val value = runCatching {
        (level as? IShipObjectWorldServerProvider)
            ?.shipObjectWorld
            ?.aerodynamicUtils
            ?.getAirTemperatureForY(y, dimensionId)
    }.getOrNull()
    return sanitizePositive(value, FALLBACK_AIR_TEMPERATURE)
}

private fun sanitizePositive(value: Double?, fallback: Double): Double {
    if (value == null || !value.isFinite() || value <= MIN_POSITIVE) {
        return fallback
    }
    return value
}
