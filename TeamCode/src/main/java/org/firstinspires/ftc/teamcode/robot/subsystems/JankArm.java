package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

public class JankArm implements AbstractSubsystem {
    // Motor that lifts the arm
    private final DcMotorEx armMotor;

    // Variable to store what angle we want the arm at
    private double targetAngle; // radians

    // Constants to manage the angle of the arm
    private static final double gearRatio = 1/2.25; // 2.25 motor rots = 1 arm rot
    private static final double encoderTicksPerRad = 751.8 / (2.0*Math.PI);
    private static final double[] angleBounds = {0.1, 4.20}; // lower, upper in radians

    // Math for going from angle to geared ticks to ticks:
    /*
    targetAngle * encoderTicksPerRad
    --------------------------------
              gearRatio
     */

    private static final double ticksPerRadian = encoderTicksPerRad / gearRatio;
    private static final int angleOffset = (int)(-(angleBounds[1] + 0.03) * ticksPerRadian); // ticks

    /**
     * Runs on each iteration of the main Op Mode loop
     */
    @Override
    public void update() {
        armMotor.setPower(0.3);

        // Quick and dirty protection just in case I try something really stupid
        if (targetAngle > angleBounds[1]) {
            targetAngle = angleBounds[1];
        } else if (targetAngle < angleBounds[0]) {
            targetAngle = angleBounds[0];
        }
        armMotor.setTargetPosition((int)round(targetAngle * ticksPerRadian) + angleOffset);
    }

    /**
     * Resets the target angle of the arm to the start position
     */
    @Override
    public void cleanup() {
        // Setting targetHeight here just in case we get one last robot update
        targetAngle = angleBounds[1];

        armMotor.setTargetPosition((int)(angleBounds[1] * ticksPerRadian) + angleOffset);
    }

    /**
     * Gets the target angle of the jank arm in radians (not the actual angle)
     * @return Target angle in inches
     */
    public double getAngle() {
        return targetAngle;
    }

    /**
     * Sets the target angle of the arm in radians
     * @param targetAngle Target angle in radians
     */
    public void setAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }


    /**
     * Initializes the janky arm subsystem
     * @param hardwareMap HardwareMap passed in from the op mode
     */
    public JankArm(HardwareMap hardwareMap) {
        // Initialize the motor
        armMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.LIFT.name);

        // Reverse the motor if we set it that way in the config
        if (HardwareNames.Motors.LIFT.reverse) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // Set it to run to a target angle and hold the angle
        armMotor.setTargetPosition((int)(-0.1 * ticksPerRadian));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the target angle
        targetAngle = angleBounds[1];
    }
}
