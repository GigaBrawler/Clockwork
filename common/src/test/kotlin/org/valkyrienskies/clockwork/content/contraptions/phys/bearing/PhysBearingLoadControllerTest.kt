package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class PhysBearingLoadControllerTest {
    @Test
    fun ship_mounted_load_defers_when_runtime_main_unresolved() {
        val resolved = PhysBearingLoadController.resolveMainIdForLoad(
            savedMainId = 42L,
            resolvedMainId = null
        )
        assertTrue(resolved.defer)
        assertEquals(42L, resolved.mainIdForJoint)
    }

    @Test
    fun ship_mounted_load_preserves_saved_main_id() {
        val unresolved = PhysBearingLoadController.resolveMainIdForLoad(
            savedMainId = 77L,
            resolvedMainId = null
        )
        assertEquals(77L, unresolved.mainIdForJoint)
        assertTrue(unresolved.defer)

        val resolved = PhysBearingLoadController.resolveMainIdForLoad(
            savedMainId = 77L,
            resolvedMainId = 88L
        )
        assertEquals(88L, resolved.mainIdForJoint)
        assertFalse(resolved.defer)
    }

    @Test
    fun world_mounted_load_allows_null_main_id() {
        val resolved = PhysBearingLoadController.resolveMainIdForLoad(
            savedMainId = null,
            resolvedMainId = null
        )
        assertFalse(resolved.defer)
        assertEquals(null, resolved.mainIdForJoint)
    }

    @Test
    fun endpoint_availability_defers_until_required_ships_exist() {
        assertTrue(
            PhysBearingLoadController.shouldDeferForEndpointAvailability(
                subShipLoaded = false,
                mainIdForJoint = null,
                mainShipLoaded = true
            )
        )

        assertTrue(
            PhysBearingLoadController.shouldDeferForEndpointAvailability(
                subShipLoaded = true,
                mainIdForJoint = 101L,
                mainShipLoaded = false
            )
        )

        assertFalse(
            PhysBearingLoadController.shouldDeferForEndpointAvailability(
                subShipLoaded = true,
                mainIdForJoint = null,
                mainShipLoaded = true
            )
        )

        assertFalse(
            PhysBearingLoadController.shouldDeferForEndpointAvailability(
                subShipLoaded = true,
                mainIdForJoint = 101L,
                mainShipLoaded = true
            )
        )
    }

    @Test
    fun chain_parent_resolution_never_requires_world_fallback() {
        val firstPass = PhysBearingLoadController.resolveMainIdForLoad(
            savedMainId = 9001L,
            resolvedMainId = null
        )
        assertTrue(firstPass.defer)
        assertEquals(9001L, firstPass.mainIdForJoint)

        val secondPass = PhysBearingLoadController.resolveMainIdForLoad(
            savedMainId = 9001L,
            resolvedMainId = 9001L
        )
        assertFalse(secondPass.defer)
        assertEquals(9001L, secondPass.mainIdForJoint)
    }
}
