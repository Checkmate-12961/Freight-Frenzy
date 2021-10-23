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
