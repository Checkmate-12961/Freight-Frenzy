package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

public class JankHand implements AbstractSubsystem {
    // Servo that does the gripping
    private final Servo gripperServo;

    // Variable to store the target position
    private double targetPosition = 1;

    /**
     * Updates the servo position based on targetPosition
     */
    @Override
    public void update() {
        gripperServo.setPosition(targetPosition);
    }

    /**
     * Runs when an Op Mode ends
     */
    @Override
    public void cleanup() {
        targetPosition = 0;
        gripperServo.setPosition(0);

        while (gripperServo.getPosition() > 0.1) { }
    }

    /**
     * Opens the jank hand
     */
    public void open() {
        targetPosition = 0;
    }

    /**
     * Closes the jank hand
     */
    public void close() {
        targetPosition = 1;
    }

    /**
     * Gets the target position of the jank hand
     * @return Target position
     */
    public double getPosition() {
        return targetPosition;
    }

    /**
     * Sets the target position of the jank hand
     * @param targetPosition Target position
     */
    public void setPosition(double targetPosition) {
        this.targetPosition = Range.clip(targetPosition, 0, 1);
    }

    /**
     * Initializes the janky hand subsystem
     * @param hardwareMap HardwareMap passed in from the op mode
     */
    public JankHand(HardwareMap hardwareMap) {
        // Initialize the servo
        gripperServo = hardwareMap.get(Servo.class, HardwareNames.Servos.GRIPPER.name);

        // Set the bounds of the servo
        gripperServo.scaleRange(HardwareNames.Servos.GRIPPER.lowerLimit, HardwareNames.Servos.GRIPPER.upperLimit);
        gripperServo.setPosition(targetPosition);
    }
}
