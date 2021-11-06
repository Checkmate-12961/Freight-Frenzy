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

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

/**
 * Subsystem to manage the carousel spinner on the front of the robot
 */
public class Carousel implements AbstractSubsystem {
    /**
     * The motor that spins the carousel
     */
    private final DcMotorEx carouselMotor;

    /**
     * Variable to store how fast we want the motor to spin
     */
    private double targetPower = 0;

    /**
     * Maximum power the motor can spin at
     */
    private static final double maxPower = 0.25;

    /**
     * Updates the power of the motor to match what we set it to
     */
    @Override
    public void update() {
        carouselMotor.setPower(targetPower * maxPower);
    }

    /**
     * Get the power of the carousel motor
     * @return The motor's power
     */
    public double getPower() {
        return targetPower;
    }

    /**
     * Set the power of the carousel motor
     * @param power The motor's power, must fit in [0, 1]
     */
    public void setPower(double power) {
        targetPower = Range.clip(power, -1, 1);
    }

    /**
     * Initialize the carousel subsystem
     * @param hardwareMap HardwareMap passed in from the op mode
     */
    public Carousel(HardwareMap hardwareMap) {
        // Initialize the servo
        carouselMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.CAROUSEL.name);

        // Reverse the motor if we set it that way in the config
        if (HardwareNames.Motors.CAROUSEL.reverse) {
            carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Set the power to 0 just in case
        carouselMotor.setPower(0);
    }

    @Override
    public void cleanup() {
        // Nothing to clean up
    }
}
