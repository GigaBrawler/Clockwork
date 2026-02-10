package org.valkyrienskies.clockwork.content.contraptions.phys.bearing

import org.joml.Vector3d

internal class PhysBearingFollowState {
    @Volatile var followAngleStalled: Boolean = false
    @Volatile var followAngleStallTicks: Int = 0
    @Volatile var followAngleStallDirSign: Int = 0
    @Volatile var lastActualAngleRadForStall: Double? = null
    @Volatile var followStallCommandGraceTicks: Int = 0
    @Volatile var followStallClearMotionTicks: Int = 0
    @Volatile var followStallPrevCmdMagDegPerTick: Double = 0.0
    @Volatile var followStallPrevCmdSign: Int = 0

    @Volatile var physServoTickCounter: Int = 0
    @Volatile var servoOmegaActualFilteredRadSec: Double = 0.0
    @Volatile var servoAlphaCmdFilteredRadSec2: Double = 0.0
    @Volatile var servoApparentInertiaScale: Double = 1.0
    @Volatile var servoPrevOmegaActualForApparentInertiaScaleRadSec: Double = Double.NaN
    @Volatile var servoPrevAppliedHingeTorqueMag: Double = 0.0

    @Volatile var tickTrackTorqueAuthority: Double = 1.0
    @Volatile var tickHoldStabilizerAuthority: Double = 1.0
    @Volatile var tickChainedDynamic: Boolean = false
    @Volatile var tickGyroRisk: Boolean = false
    @Volatile var tickFollowModeActive: Boolean = false
    @Volatile var tickFollowBrakePhase: Boolean = false
    @Volatile var tickFollowHoldPhase: Boolean = false
    @Volatile var tickFollowRestStable: Boolean = false
    @Volatile var tickFollowRigidRest: Boolean = false
    @Volatile var tickFollowRigidRestBlend: Double = 0.0
    @Volatile var tickFollowHingeMicroSuppressed: Boolean = false
    @Volatile var tickFollowSeatMicroSuppressed: Boolean = false
    @Volatile var tickFollowTiltMicroSuppressed: Boolean = false
    @Volatile var tickFollowSeatRestBandSatisfied: Boolean = false
    @Volatile var tickFollowTiltRestBandSatisfied: Boolean = false
    @Volatile var tickFollowRestUltraStable: Boolean = false
    @Volatile var tickFollowUltraSleep: Boolean = false

    @Volatile var omegaEmaRadSec: Double = 0.0
    @Volatile var appInertiaConsistentTicks: Int = 0
    @Volatile var followRestStableTicks: Int = 0
    @Volatile var followRestUltraStableTicks: Int = 0
    @Volatile var followUltraSleepTicks: Int = 0
    @Volatile var followRigidRestTicks: Int = 0
    @Volatile var followHingeMicroTicks: Int = 0
    @Volatile var followSeatMicroTicks: Int = 0
    @Volatile var followTiltMicroTicks: Int = 0
    @Volatile var followHoldAcquireTicksRemaining: Int = 0
    @Volatile var followHoldSettleTicksRemaining: Int = 0
    @Volatile var followSpawnSettleTicksRemaining: Int = 0
    @Volatile var followSeatBiasFreezeTicks: Int = 0
    @Volatile var followTiltBiasFreezeTicks: Int = 0
    @Volatile var followHoldCaptureTicks: Int = 0
    @Volatile var followHoldEmergencyReleaseTicks: Int = 0
    @Volatile var followHoldRingDownTicks: Int = 0
    @Volatile var followHoldLastErrorSign: Int = 0
    @Volatile var followWasTrackLastTick: Boolean = false
    @Volatile var followBrakeHoldAngleRad: Double? = null

    @Volatile var followSeatHoldBiasAccWorld: Vector3d = Vector3d()
    @Volatile var followTiltHoldBiasAlphaWorld: Vector3d = Vector3d()
    @Volatile var followPrevSeatBiasErrorWorld: Vector3d = Vector3d()
    @Volatile var followPrevTiltBiasErrorWorld: Vector3d = Vector3d()

    fun resetStallState() {
        followAngleStalled = false
        followAngleStallTicks = 0
        followAngleStallDirSign = 0
        lastActualAngleRadForStall = null
        followStallCommandGraceTicks = 0
        followStallClearMotionTicks = 0
        followStallPrevCmdMagDegPerTick = 0.0
        followStallPrevCmdSign = 0
    }

    fun resetServoTransientState() {
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
        tickFollowUltraSleep = false

        followRestStableTicks = 0
        followRestUltraStableTicks = 0
        followUltraSleepTicks = 0
        followRigidRestTicks = 0
        followHingeMicroTicks = 0
        followSeatMicroTicks = 0
        followTiltMicroTicks = 0
        followHoldAcquireTicksRemaining = 0
        followHoldSettleTicksRemaining = 0
        followSpawnSettleTicksRemaining = 0
        followSeatBiasFreezeTicks = 0
        followTiltBiasFreezeTicks = 0
        followHoldCaptureTicks = 0
        followHoldEmergencyReleaseTicks = 0
        followHoldRingDownTicks = 0
        followHoldLastErrorSign = 0
        followWasTrackLastTick = false
        followBrakeHoldAngleRad = null

        followSeatHoldBiasAccWorld = Vector3d()
        followTiltHoldBiasAlphaWorld = Vector3d()
        followPrevSeatBiasErrorWorld = Vector3d()
        followPrevTiltBiasErrorWorld = Vector3d()
    }

    fun resetAllRuntimeState() {
        resetStallState()
        resetServoTransientState()
    }
}
