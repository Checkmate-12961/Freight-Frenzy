package org.firstinspires.ftc.teamcode.robot.abstracts

import android.util.Log
import org.firstinspires.ftc.teamcode.robot.abstracts.exceptions.SubsystemException

/**
 * Custom map to store subsystems.
 *
 * @property tag Callback to get the tag for logging.
 */
class SubsystemMap(private val tag: () -> String) {
    private val map: MutableMap<String, AbstractSubsystem> = mutableMapOf()
    private val regex = Regex("([a-z]|[A-Z])([a-z]|[A-Z]|[0-9])*")

    /**
     * Keys in the map.
     *
     * @see MutableMap.keys
     */
    val keys: MutableSet<String>
        get() = map.keys

    /**
     * Values in the map.
     *
     * @see MutableMap.values
     */
    val values: MutableCollection<AbstractSubsystem>
        get() = map.values

    /**
     * Set a value in the map.
     *
     * @param id Key to set the map at.
     * @param subsystem Value to set it to.
     *
     * @see MutableMap.set
     */
    operator fun set(id: String, subsystem: AbstractSubsystem) {
        if (id matches regex) {
            map[id] = subsystem
            Log.i(tag.invoke(), "Added subsystem $id")
        } else {
            throw SubsystemException("$id is not a valid subsystem name.")
        }
    }

    /**
     * Get a value in the map.
     *
     * @param id Key of the value.
     * @return The value.
     *
     * @see MutableMap.get
     */
    operator fun get(id: String): AbstractSubsystem? {
        return map[id]
    }

    /**
     * Register a subsystem.
     *
     * @param subsystem Subsystem to register.
     */
    fun register(subsystem: AbstractSubsystem) {
        this[subsystem.javaClass.simpleName] = subsystem
    }

    /**
     * De-register a subsystem.
     *
     * @param id Simple name of the subsystem to de-register.
     */
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