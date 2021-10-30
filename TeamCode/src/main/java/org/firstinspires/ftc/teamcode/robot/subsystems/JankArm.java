package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

public class JankArm implements AbstractSubsystem {
    // Motor that lifts the arm
    private final DcMotorEx armMotor;

    // Enum to switch between power and angle modes
    public enum ArmMode {
        POWER, ANGLE
    }

    // Variable to store the current mode
    private ArmMode mode;

    // Variable to store what angle we want the arm at
    private double targetAngle; // radians

    // Variable to store the target power of the arm
    private double targetPower = 0;

    // Constants to manage the power of the arm
    private static final double powerCoefficient = 0.3;

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
        // Do different things depending on the mode
        switch (mode) {
            // If we're in power mode, set the power of the arm to the target power
            case POWER:
                targetPower = Range.clip(targetPower, -1, 1);
                armMotor.setPower(targetPower * powerCoefficient);
                break;
            // If we're in angle mode, set the target position of the arm to the target angle
            case ANGLE:
                armMotor.setPower(powerCoefficient);

                // Quick and dirty protection just in case I try something really stupid
                targetAngle = Range.clip(targetAngle, angleBounds[0], angleBounds[1]);

                armMotor.setTargetPosition((int)round(targetAngle * ticksPerRadian) + angleOffset);
        }
    }

    /**
     * Resets the target angle of the arm to the start position
     */
    @Override
    public void cleanup() {
        // Setting targetHeight here just in case we get one last robot update
        targetAngle = angleBounds[1];

        armMotor.setTargetPosition((int)(angleBounds[1] * ticksPerRadian) + angleOffset);

        while (armMotor.isBusy()) { }
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
     * Sets the target power of the arm
     * @param targetPower Target power of the arm
     */
    public void setPower(double targetPower) {
        this.targetPower = targetPower;
    }

    /**
     * Gets the target power of the arm
     * @return Target power of the arm
     */
    public double getPower() {
        return targetPower;
    }

    /**
     * Sets the mode of the arm to power or angle
     */
    public void setMode(ArmMode mode) {
        this.mode = mode;
        targetPower = 0;

        switch (mode) {
            case POWER:
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case ANGLE:
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetAngle = armMotor.getCurrentPosition() / ticksPerRadian;
                break;
        }
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
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the mode to angle
        setMode(ArmMode.ANGLE);
        targetAngle = angleBounds[1];
    }
}
