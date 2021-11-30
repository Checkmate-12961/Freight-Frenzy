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

import static java.lang.Math.round;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Subsystem to manage the lift on the front of the robot
 */
@Config
public class Lift implements AbstractSubsystem {
    // Motor that does the lifting
    private final DcMotorEx liftMotor;

    // Variables to store the target positions of the lift
    public static double targetLow = 5;
    public static double targetMid = 10;
    public static double targetHigh = 15;

    // Double suppliers because enum
    private static double getTargetRest() { return 0; }
    private static double getTargetLow() { return targetLow; }
    private static double getTargetMid() { return targetMid; }
    private static double getTargetHigh() { return targetHigh; }

    public enum SetPoints {
        LOW(Lift::getTargetLow),
        MID(Lift::getTargetMid),
        HIGH(Lift::getTargetHigh),
        REST(Lift::getTargetRest);

        private final DoubleSupplier doubleSupplier;

        public double getValue() {
            return doubleSupplier.getAsDouble();
        }

        SetPoints(DoubleSupplier doubleSupplier) {
            this.doubleSupplier = doubleSupplier;
        }
    }

    // Variable to store how high up we want the lift to be in inches
    private double targetHeight; // inches

    // Constants to manage the height of the lift
    private static final double spoolDiameter = 1.5; // inches
    private static final double encoderTicksPerRev = 537.7;
    private static final double maxHeight = 20000.0; // inches

    // Math for going from height to revolutions to geared ticks to ticks:
    /*
    targetHeight * encoderTicksPerRev
    ---------------------------------
          spoolDiameter * Math.PI
     */

    private static final double ticksPerInch = encoderTicksPerRev / (spoolDiameter * Math.PI);

    /**
     * Gets the target height of the lift in inches (not the actual height)
     * @return Target height in inches
     */
    public double getTargetHeight() {
        return targetHeight;
    }

    /**
     * Gets the real height of the lift in inches
     * @return Real height in inches
     */
    public double getHeight() {
        return liftMotor.getCurrentPosition() / ticksPerInch;
    }

    /**
     * Sets the target height of the lift in inches
     * @param targetHeight Target height in inches
     */
    public void setHeight(double targetHeight) {
        this.targetHeight = Range.clip(targetHeight, 0, maxHeight);
        liftMotor.setTargetPosition((int)round(this.targetHeight * ticksPerInch));
    }

    /**
     * Sets the target point of the lift
     */
    public void setTarget(SetPoints target) {
        setHeight(target.getValue());
    }

    /**
     * Initializes the lift subsystem
     * @param hardwareMap HardwareMap passed in from the op mode
     */
    public Lift(HardwareMap hardwareMap) {
        // Initialize the motor
        liftMotor = hardwareMap.get(DcMotorEx.class, Motors.LIFT.name);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse the motor if we set it that way in the config
        if (Motors.LIFT.reverse) {
            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // Set it to run to a target position and hold the position
        setHeight(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(1);
    }
}
