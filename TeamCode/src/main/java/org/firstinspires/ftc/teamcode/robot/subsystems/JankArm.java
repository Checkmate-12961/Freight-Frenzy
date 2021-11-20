package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.round;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

@Config
public class JankArm implements AbstractSubsystem {
    // Motor that lifts the arm
    private final DcMotorEx armMotor;

    // Constants to manage the power of the arm
    public static double powerCoefficient = 1;
    private double lastPowerCoefficient = powerCoefficient;

    // PIDF
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;

    private double targetAngle = 0;

    // Constants to manage the angle of the arm
    private static final double gearRatio = 1/2.25; // 2.25 motor rots = 1 arm rot
    private static final double encoderTicksPerRad = 751.8 / (2.0*Math.PI);

    public static double lowerAngleBound = 0;
    public static double upperAngleBound = 4.20;

    // Math for going from angle to geared ticks to ticks:
    /*
    targetAngle * encoderTicksPerRad
    --------------------------------
              gearRatio
     */

    private static final double ticksPerRadian = encoderTicksPerRad / gearRatio;

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
        this.targetAngle = Range.clip(targetAngle, lowerAngleBound, upperAngleBound);
        armMotor.setTargetPosition((int) round(this.targetAngle * ticksPerRadian));
    }

    /**
     * Floppa or un-floppa the arm
     * @param enable If true, floppa the arm
     */
    public void floppa(boolean enable) {
        if (enable) {
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            setAngle(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void update() {
        if (powerCoefficient != lastPowerCoefficient) {
            lastPowerCoefficient = powerCoefficient;
            armMotor.setPower(powerCoefficient);
        }
    }

    /**
     * Initializes the janky arm subsystem
     * @param hardwareMap HardwareMap passed in from the op mode
     */
    public JankArm(HardwareMap hardwareMap) {
        // Initialize the motor
        armMotor = hardwareMap.get(DcMotorEx.class, Motors.LIFT.name);

        // Reverse the motor if we set it that way in the config
        if (Motors.LIFT.reverse) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // Set it to run to a target angle and hold the angle
        armMotor.setTargetPosition(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PIDFCoefficients PIDFValues = armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        kP = PIDFValues.p;
        kI = PIDFValues.i;
        kD = PIDFValues.d;
        kF = PIDFValues.f;

        armMotor.setPower(powerCoefficient);
    }
}
