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

/**
 * Class that all robots need to extend
 */
abstract class AbstractRobot {
    private val subsystems: ArrayList<AbstractSubsystem> = arrayListOf()

    /**
     * Updates all subsystems
     */
    fun update() {
        for (subsystem in subsystems) {
            subsystem.update()
        }
    }

    /**
     * Runs cleanup() on all subsystems
     */
    fun cleanup() {
        for (subsystem in subsystems) {
            subsystem.cleanup()
        }
    }

    /**
     * Adds a new subsystem
     * @param subsystem Subsystem to add. Must implement the Subsystem class
     */
    protected fun addSubsystem(subsystem: AbstractSubsystem) {
        subsystems.add(subsystem)
    }

}