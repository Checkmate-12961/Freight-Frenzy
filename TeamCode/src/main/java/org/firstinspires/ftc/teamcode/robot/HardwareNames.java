package org.firstinspires.ftc.teamcode.robot;

/**
 * Class to manage names of hardware connected to the robot
 */
public class HardwareNames {
    public enum Motors {
        RIGHT_FRONT ("rightFront", false),
        RIGHT_REAR ("rightRear", false),
        LEFT_FRONT ("leftFront", true),
        LEFT_REAR ("leftRear", true);

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
