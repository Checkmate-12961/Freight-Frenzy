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

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.subsystems.Carousel.Carousel.maxPower

/**
 * Subsystem to manage the carousel spinner on the front of the robot
 * @param hardwareMap HardwareMap passed in from the op mode
 */
class Carousel(hardwareMap: HardwareMap) : AbstractSubsystem {
    /**
     * The motor that spins the carousel
     */
    private val carouselMotor: DcMotorEx

    var power: Double
        /**
         * Get the power of the carousel motor
         * @return The motor's power
         */
        get() = carouselMotor.power
        /**
         * Set the power of the carousel motor
         * @param power The motor's power, must fit in [0, 1]
         */
        set(power) {
            carouselMotor.power = Range.clip(power, -1.0, 1.0) * maxPower
        }

    @Config
    object Carousel {
        /**
         * Maximum power the motor can spin at
         */
        @JvmField var maxPower = 0.5
    }

    /**
     * Initialize the carousel subsystem
     */
    init {
        // Initialize the servo
        carouselMotor = hardwareMap.get(DcMotorEx::class.java, Motors.CAROUSEL.id)

        // Reverse the motor if we set it that way in the config
        if (Motors.CAROUSEL.reverse) {
            carouselMotor.direction = DcMotorSimple.Direction.REVERSE
        }

        // Set the power to 0 just in case
        carouselMotor.power = 0.0
    }
}