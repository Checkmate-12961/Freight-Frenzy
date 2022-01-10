/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.robot.abstracts

import android.util.Log
import org.firstinspires.ftc.teamcode.robot.abstracts.exceptions.SubsystemException

/**
 * Class that all robots need to extend
 */
abstract class AbstractRobot {
    protected abstract val tag: String
    val subsystems = SubsystemMap{ tag }

    /**
     * Runs once before the loop
     */
    fun preLoop() {
        for (subsystem in subsystems.values) {
            subsystem.preLoop()
        }
    }

    /**
     * Updates all subsystems
     */
    fun update() {
        for (subsystem in subsystems.values) {
            subsystem.update()
        }
    }

    /**
     * Runs cleanup() on all subsystems
     */
    fun cleanup() {
        for (subsystem in subsystems.values) {
            subsystem.cleanup()
        }
    }

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
}