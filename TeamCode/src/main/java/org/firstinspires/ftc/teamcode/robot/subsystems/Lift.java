package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

/**
 * Subsystem to manage the lift on the front of the robot
 */
public class Lift implements AbstractSubsystem {
    // Motor that does the lifting
    private final DcMotorEx liftMotor;

    // Variable to store how high up we want the lift to be in inches
    private double targetHeight; // inches

    // Constants to manage the height of the lift
    private static final double spoolDiameter = 1.5; // inches
    private static final double encoderTicksPerRev = 751.8;
    private static final double maxHeight = 21.0; // inches

    // Math for going from height to revolutions to ticks:
    /*
    targetHeight * encoderTicksPerRev
    ---------------------------------
          spoolDiameter * Math.PI
     */

    private static final double heightToTicksMultiplier = encoderTicksPerRev / (spoolDiameter * Math.PI);

    /**
     * Updates the target height of the lift based on targetHeight
     */
    @Override
    public void update() {
        liftMotor.setPower(1);

        // Quick and dirty protection just in case I try something really stupid
        if (targetHeight > maxHeight) {
            targetHeight = maxHeight;
        } else if (targetHeight < 0) {
            targetHeight = 0;
        }
        liftMotor.setTargetPosition((int)round(targetHeight * heightToTicksMultiplier));
    }

    /**
     * Resets the target height of the lift to 0
     */
    @Override
    public void cleanup() {
        // Setting targetHeight here just in case we get one last robot update
        targetHeight = 0;

        liftMotor.setTargetPosition(0);
    }

    /**
     * Gets the target height of the lift in inches (not the actual height)
     * @return Target height in inches
     */
    public double getHeight() {
        return targetHeight;
    }

    /**
     * Sets the target height of the lift in inches
     * @param targetHeight Target height in inches
     */
    public void setHeight(double targetHeight) {
        this.targetHeight = targetHeight;
    }

    /**
     * Initializes the lift subsystem
     * @param hardwareMap HardwareMap passed in from the op mode
     */
    public Lift(HardwareMap hardwareMap) {
        // Initialize the motor
        liftMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.LIFT.name);

        // Reverse the motor if we set it that way in the config
        if (HardwareNames.Motors.LIFT.reverse) {
            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // Set it to run to a target position and hold the position
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the target height
        targetHeight = 0;
    }
}
