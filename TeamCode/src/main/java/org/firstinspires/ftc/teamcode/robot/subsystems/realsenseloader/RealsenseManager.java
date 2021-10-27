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

package org.firstinspires.ftc.teamcode.robot.subsystems.realsenseloader;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.robot.util.PoseUtil;

import javax.annotation.Nullable;

public class RealsenseManager {
    // TODO: tune this based on the relative positions of the camera and the robot
    //  This transform is the distance between the camera and the robot (not vice versa)
    //  Use the normal coordinate system the field uses.
    public static Transform2d cameraRobotOffset = PoseUtil.toTransform2d(
            PoseUtil.inchesToMeters(
                    new Pose2d(-9.0, -8.25)));
    public static double encoderMeasurementCovariance = 0.8;

    @Nullable
    public static T265Camera slamera = null;

    public static void init(HardwareMap hardwareMap) {
        if (slamera == null) {
            slamera = new T265Camera(cameraRobotOffset, encoderMeasurementCovariance, hardwareMap.appContext);
        }
        slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d());
        slamera.start();
    }

    public static void cleanup() {
        if (slamera != null) {
            slamera.stop();
        }
    }
}
