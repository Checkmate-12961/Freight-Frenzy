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
package org.firstinspires.ftc.teamcode.robot

/**
 * Class to manage names of hardware connected to the robot
 */
class HardwareNames {
    /**
     * Contains hardware info for motors
     */
    enum class Motors(val id: String, val reverse: Boolean) {

        // Drivetrain
        RIGHT_FRONT("rightFront", false),
        RIGHT_REAR("rightRear", false),
        LEFT_FRONT("leftFront",true),
        LEFT_REAR("leftRear", true),
        // Carousel mechanism
        CAROUSEL("carousel", false),
        // Intake mechanism
        INTAKE("intake", true),
        // Lift mechanism
        LIFT("lift", false);
    }

    enum class Encoders(val id: String, val reverse: Boolean) {
        LEFT("leftRear", true),
        FRONT("leftFront", false),
        RIGHT("rightRear", false)
    }

    /**
     * Contains hardware info for servos
     */
    enum class Servos(val id: String, val reversed: Boolean) {
        BUCKET("bucket", false),
        CAP_SHOULDER("capShoulder", false),
        CAP_ELBOW("capElbow", false)
    }

    /**
     * Contains hardware info for continuous rotation servos
     */
    enum class CRServos(val id: String, val reverse: Boolean) {
        DUMMY("", false)
    }

    /**
     * Contains hardware info for cameras
     */
    enum class Cameras(val id: String) {
        WEBCAM("webcam")
    }
}