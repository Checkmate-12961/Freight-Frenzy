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

package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

/**
 * Bucket thingy on the lift. There is no direct access to the servo position for a reason.
 */
class Bucket(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "Bucket"
    override val subsystems = SubsystemMap{ tag }

    private var bucketServo = hardwareMap.get(Servo::class.java, Servos.BUCKET.id)

    /**
     * Set positions for the bucket.
     *
     * All of these values are derived from dinking around.
     *  DO NOT TOUCH THEM OR SIMON WILL TAKE YOUR KNEECAPS.
     */
    enum class Positions(@JvmField val position: Double) {
        // Position when the bucket is dumping
        DUMP(0.42),
        // The normal position
        REST(0.21),
        // The position when the bucket is all the way down
        ZERO(0.14)
    }

    /**
     * The position the bucket is at
     *
     * These aren't raw double values, they're values on the enum so that programmers don't break
     *  things
     */
    var position: Positions = Positions.REST
        set(value) {
            field = value
            bucketServo.position = value.position
        }

    init {
        // Grab the servo from the hardware map

        // If the servo is reversed in the config, reverse it
        if (Servos.BUCKET.reversed) {
            bucketServo.direction = Servo.Direction.REVERSE
        }

        // Set the position to the rest position
        bucketServo.position = Positions.REST.position
    }
}