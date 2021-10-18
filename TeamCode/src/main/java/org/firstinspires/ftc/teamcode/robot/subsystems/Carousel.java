package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

/**
 * Subsystem to manage the carousel spinner on the front of the robot
 */
public class Carousel implements AbstractSubsystem {
    // Servo
    private final CRServo carouselMotor;

    // Variable to store how fast we want the motor to spin
    private double targetPower = 0;

    // Motor power is multiplied by this before it's set. Must fit in (0, 1]
    private static final double powerMultiplier = 1;

    /**
     * Updates the power of the motor to match what we set it to
     */
    @Override
    public void update() {
        carouselMotor.setPower(targetPower * powerMultiplier);
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
        carouselMotor = hardwareMap.get(CRServo.class, HardwareNames.CRServos.CAROUSEL.name);

        // Reverse the motor if we set it that way in the config
        if (HardwareNames.CRServos.CAROUSEL.reverse) {
            carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Set the power to 0 just in case
        carouselMotor.setPower(0);
    }
}
