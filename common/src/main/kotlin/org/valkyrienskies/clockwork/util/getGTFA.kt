package org.valkyrienskies.clockwork.util

import net.minecraft.core.BlockPos
import net.minecraft.server.level.ServerLevel
import org.valkyrienskies.core.internal.joints.VSJoint
import org.valkyrienskies.core.internal.joints.VSJointAndId
import org.valkyrienskies.mod.common.ValkyrienSkiesMod
import org.valkyrienskies.mod.common.dimensionId
import org.valkyrienskies.mod.common.util.GameToPhysicsAdapter
import java.util.UUID
import java.util.function.Consumer

val ServerLevel.gtpa: GameToPhysicsAdapter get() = ValkyrienSkiesMod.getOrCreateGTPA(this.dimensionId)
fun GameToPhysicsAdapter.updateJoint(id: Int, joint: VSJoint) { this.updateJoint(VSJointAndId(id, joint)) }

fun newPersistentJointKey(): String = UUID.randomUUID().toString()

fun buildPersistentOwnerRef(dimensionId: String, blockPos: BlockPos, slot: String): String {
    return "$dimensionId|${blockPos.x}|${blockPos.y}|${blockPos.z}|$slot"
}

fun GameToPhysicsAdapter.addJointPersistent(
    joint: VSJoint,
    ownerType: String? = null,
    ownerRef: String? = null,
    persistentKey: String? = null,
    delay: Int = 0,
    function: (Int) -> Unit
) {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "addJointPersistent" && it.parameterCount == 6
    }
    if (reflectiveMethod != null) {
        reflectiveMethod.invoke(
            this,
            joint,
            ownerType,
            ownerRef,
            persistentKey,
            delay,
            Consumer<Int> { function(it) }
        )
        return
    }
    this.addJoint(joint, delay) { function(it) }
}

fun GameToPhysicsAdapter.updateJointPersistent(runtimeOrLegacyId: Int, joint: VSJoint) {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "updateJointPersistent" && it.parameterCount == 2
    }
    if (reflectiveMethod != null) {
        reflectiveMethod.invoke(this, runtimeOrLegacyId, joint)
        return
    }
    this.updateJoint(runtimeOrLegacyId, joint)
}

fun GameToPhysicsAdapter.removeJointPersistent(runtimeOrLegacyId: Int) {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "removeJointPersistent" && it.parameterCount == 1
    }
    if (reflectiveMethod != null) {
        reflectiveMethod.invoke(this, runtimeOrLegacyId)
        return
    }
    this.removeJoint(runtimeOrLegacyId)
}

fun GameToPhysicsAdapter.resolveRuntimeJointId(runtimeOrLegacyId: Int): Int {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "resolveRuntimeJointId" && it.parameterCount == 1
    }
    return if (reflectiveMethod != null) {
        (reflectiveMethod.invoke(this, runtimeOrLegacyId) as? Int) ?: runtimeOrLegacyId
    } else {
        runtimeOrLegacyId
    }
}

fun GameToPhysicsAdapter.bindPersistentKey(persistentKey: String, runtimeId: Int) {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "bindPersistentKey" && it.parameterCount == 2
    }
    if (reflectiveMethod != null) {
        reflectiveMethod.invoke(this, persistentKey, runtimeId)
    }
}
