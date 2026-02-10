package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

internal object PhysBearingStabilizerController {
    data class HoldContext(
        val followMode: Boolean,
        val followBrakePhase: Boolean,
        val followHoldPhase: Boolean,
        val followRestUltraStable: Boolean,
        val followUltraSleep: Boolean,
        val followRigidRest: Boolean,
        val followRigidRestBlend: Double,
        val followHoldSettleActive: Boolean,
        val followAcquireActive: Boolean
    ) {
        val inFollowHold: Boolean
            get() = followMode && followHoldPhase

        val inFollowBrakeOrHold: Boolean
            get() = followMode && (followBrakePhase || followHoldPhase)
    }

    data class FloorRequest(
        val seatAuthorityFloorMapped: Double,
        val tiltAuthorityFloorMapped: Double,
        val seatKpFloorScaleMapped: Double,
        val tiltStiffnessFloorMapped: Double,
        val offAxisDampingZetaMinMapped: Double,
        val rigidBlend01: Double
    )

    fun resolveAxisProfile(
        followMode: Boolean,
        followHoldPhase: Boolean,
        absAxisY: Double,
        strength01: Double
    ): PhysBearingServoMath.AxisHoldProfile {
        if (!followMode || !followHoldPhase) return PhysBearingServoMath.AxisHoldProfile.IDENTITY
        return PhysBearingServoMath.computeAxisHoldProfile(absAxisY, strength01)
    }

    fun resolveFixedHoldFloors(
        context: HoldContext,
        request: FloorRequest
    ): PhysBearingServoMath.FixedHoldFloorProfile {
        if (!context.inFollowHold) return PhysBearingServoMath.FixedHoldFloorProfile.IDENTITY
        return PhysBearingServoMath.applyFixedHoldFloorProfile(
            seatAuthorityFloorMapped = request.seatAuthorityFloorMapped,
            tiltAuthorityFloorMapped = request.tiltAuthorityFloorMapped,
            seatKpFloorScaleMapped = request.seatKpFloorScaleMapped,
            tiltStiffnessFloorMapped = request.tiltStiffnessFloorMapped,
            offAxisDampingZetaMinMapped = request.offAxisDampingZetaMinMapped,
            rigidBlend01 = if (context.followRigidRest) context.followRigidRestBlend else 0.0
        )
    }

    fun resolveHoldSupportMode(
        context: HoldContext,
        biasActive: Boolean
    ): PhysBearingServoMath.HoldSupportMode {
        return PhysBearingServoMath.computeHoldSupportMode(
            inFollowHold = context.inFollowHold,
            ultraSleep = context.followUltraSleep,
            restUltraStable = context.followRestUltraStable,
            settleActive = context.followHoldSettleActive,
            acquireActive = context.followAcquireActive,
            biasActive = biasActive
        )
    }
}
