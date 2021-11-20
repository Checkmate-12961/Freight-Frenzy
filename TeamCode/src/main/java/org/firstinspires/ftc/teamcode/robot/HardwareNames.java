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

        // Carousel mechanism
        CAROUSEL ("carousel", false),

        // Lift mechanism
        LIFT ("lift", true);

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

    /**
     * Contains hardware info for servos
     */
    public enum Servos {
        GRIPPER ("gripper", 0, 1);

        public final String name;
        public final double upperLimit;
        public final double lowerLimit;

        Servos(String name, double lowerLimit, double upperLimit){
            this.name = name;
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;
        }
    }

    /**
     * Contains hardware info for continuous rotation servos
     */
    public enum CRServos {
        DUMMY ("", false);

        public final String name;
        public final boolean reverse;

        CRServos(String name, boolean reverse) {
            this.name = name;
            this.reverse = reverse;
        }
    }
}
