package org.valkyrienskies.clockwork.util

import net.minecraft.core.BlockPos
import net.minecraft.server.level.ServerLevel
import org.valkyrienskies.core.internal.joints.VSJoint
import org.valkyrienskies.core.internal.joints.VSJointAndId
import org.valkyrienskies.mod.common.ValkyrienSkiesMod
import org.valkyrienskies.mod.common.dimensionId
import org.valkyrienskies.mod.common.util.GameToPhysicsAdapter
import java.nio.charset.StandardCharsets
import java.util.UUID
import java.util.function.Consumer

val ServerLevel.gtpa: GameToPhysicsAdapter get() = ValkyrienSkiesMod.getOrCreateGTPA(this.dimensionId)
fun GameToPhysicsAdapter.updateJoint(id: Int, joint: VSJoint) { this.updateJoint(VSJointAndId(id, joint)) }

fun newPersistentJointKey(): String = UUID.randomUUID().toString()

fun buildPersistentOwnerRef(dimensionId: String, blockPos: BlockPos, slot: String): String {
    return "$dimensionId|${blockPos.x}|${blockPos.y}|${blockPos.z}|$slot"
}

private fun compareBlockPos(a: BlockPos, b: BlockPos): Int {
    if (a.x != b.x) return a.x.compareTo(b.x)
    if (a.y != b.y) return a.y.compareTo(b.y)
    return a.z.compareTo(b.z)
}

fun buildCanonicalPairOwnerRef(dimensionId: String, posA: BlockPos, posB: BlockPos, slot: String): String {
    val (first, second) = if (compareBlockPos(posA, posB) <= 0) {
        posA to posB
    } else {
        posB to posA
    }
    return "pair:$dimensionId|${first.x},${first.y},${first.z}|${second.x},${second.y},${second.z}|$slot"
}

fun deterministicPersistentJointKey(ownerRef: String): String {
    return UUID.nameUUIDFromBytes(ownerRef.toByteArray(StandardCharsets.UTF_8)).toString()
}

fun isCanonicalPairLeader(selfPos: BlockPos, otherPos: BlockPos): Boolean {
    return compareBlockPos(selfPos, otherPos) <= 0
}

fun buildPersistentKeyOwnerRef(persistentKey: String): String {
    return "key:$persistentKey"
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

fun GameToPhysicsAdapter.getRuntimeIdForPersistentKey(persistentKey: String): Int? {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "getRuntimeIdForPersistentKey" && it.parameterCount == 1
    }
    return if (reflectiveMethod != null) {
        reflectiveMethod.invoke(this, persistentKey) as? Int
    } else {
        null
    }
}

fun GameToPhysicsAdapter.getPersistentKeyForRuntimeId(runtimeId: Int): String? {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "getPersistentKeyForRuntimeId" && it.parameterCount == 1
    }
    return if (reflectiveMethod != null) {
        reflectiveMethod.invoke(this, runtimeId) as? String
    } else {
        null
    }
}

fun GameToPhysicsAdapter.hasPersistentKeyBinding(persistentKey: String): Boolean {
    val reflectiveMethod = this::class.java.methods.firstOrNull {
        it.name == "hasPersistentKeyBinding" && it.parameterCount == 1
    }
    return if (reflectiveMethod != null) {
        (reflectiveMethod.invoke(this, persistentKey) as? Boolean) == true
    } else {
        getRuntimeIdForPersistentKey(persistentKey) != null
    }
}
