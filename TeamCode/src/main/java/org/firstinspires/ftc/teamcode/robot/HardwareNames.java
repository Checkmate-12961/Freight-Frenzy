package org.firstinspires.ftc.teamcode.robot;

/**
 * Class to manage names of hardware connected to the robot
 */
public class HardwareNames {
    /**
     * Contains hardware info for motors
     */
    public enum Motors {
        // Drivetrain
        RIGHT_FRONT ("rightFront", false),
        RIGHT_REAR ("rightRear", false),
        LEFT_FRONT ("leftFront", true),
        LEFT_REAR ("leftRear", true),

        // Lift mechanism
        LIFT ("lift", false);

        public final String name;
        public final boolean reverse;

        Motors(String name, boolean reverse){
            this.name = name;
            this.reverse = reverse;
        }
    }

    public enum Encoders {
        LEFT ("leftRear", true),
        FRONT ("leftFront", false),
        RIGHT ("rightRear", false);

        public final String name;
        public final boolean reverse;

        Encoders(String name, boolean reverse){
            this.name = name;
            this.reverse = reverse;
        }
    }
}
