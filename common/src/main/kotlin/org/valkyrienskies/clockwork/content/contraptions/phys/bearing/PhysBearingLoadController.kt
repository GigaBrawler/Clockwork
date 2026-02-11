package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

internal object PhysBearingLoadController {
    data class MainIdResolution(
        val mainIdForJoint: Long?,
        val defer: Boolean
    )

    fun resolveMainIdForLoad(savedMainId: Long?, resolvedMainId: Long?): MainIdResolution {
        val mainIdForJoint = resolvedMainId ?: savedMainId
        // Ship-mounted bearings must not temporarily degrade to world-mounted during load.
        val defer = savedMainId != null && resolvedMainId == null
        return MainIdResolution(mainIdForJoint = mainIdForJoint, defer = defer)
    }

    fun shouldDeferForEndpointAvailability(
        subShipLoaded: Boolean,
        mainIdForJoint: Long?,
        mainShipLoaded: Boolean
    ): Boolean {
        if (!subShipLoaded) return true
        if (mainIdForJoint != null && !mainShipLoaded) return true
        return false
    }
}
