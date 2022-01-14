package org.firstinspires.ftc.teamcode.robot.abstracts

import android.util.Log
import org.firstinspires.ftc.teamcode.robot.abstracts.exceptions.SubsystemException

class SubsystemMap(private val tag: () -> String) {
    private val map: MutableMap<String, AbstractSubsystem> = mutableMapOf()
    private val regex = Regex("([a-z]|[A-Z])([a-z]|[A-Z]|[0-9])*")

    val keys: MutableSet<String>
        get() = map.keys

    val values: MutableCollection<AbstractSubsystem>
        get() = map.values

    operator fun set(id: String, subsystem: AbstractSubsystem) {
        if (id matches regex) {
            map[id] = subsystem
            Log.i(tag.invoke(), "Added subsystem $id")
        } else {
            throw SubsystemException("$id is not a valid subsystem name.")
        }
    }

    operator fun get(id: String): AbstractSubsystem? {
        return map[id]
    }

    fun register(subsystem: AbstractSubsystem) {
        this[subsystem.javaClass.simpleName] = subsystem
    }

    fun deregister(id: String) {
        if (id matches regex) {
            map.remove(id)
            try {
                map[id]?.cleanup()
            } catch (e: Error) {
                Log.e(tag.invoke(), e.toString())
            }
            Log.i(tag.invoke(), "Removed subsystem $id")
        } else {
            throw SubsystemException("$id is not a valid subsystem name.")
        }
    }
}