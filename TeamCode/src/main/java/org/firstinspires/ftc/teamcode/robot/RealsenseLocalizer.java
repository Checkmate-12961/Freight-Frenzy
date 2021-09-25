package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.robot.util.Pose2dConversionUtil;

public class RealsenseLocalizer implements Localizer {
    // TODO: tune this based on the relative positions of the camera and the robot
    public static Transform2d cameraRobotOffset = new Transform2d(new Translation2d(9.0,8.25), new Rotation2d());
    public double encoderMeasurementCovariance = 0.8;

    private Pose2d lastPose = null;

    private final T265Camera slamera;

    public RealsenseLocalizer(HardwareMap hardwareMap) {
        super();
        slamera = new T265Camera(cameraRobotOffset, encoderMeasurementCovariance, hardwareMap.appContext);
        slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d());
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return Pose2dConversionUtil.toRoadrunnerPose(slamera.getLastReceivedCameraUpdate().pose);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        slamera.setPose(Pose2dConversionUtil.toRealsensePose(pose));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        if (lastPose != null) {
            return getPoseEstimate().minus(lastPose);
        } else {
            return null;
        }
    }

    @Override
    public void update() {
        lastPose = getPoseEstimate();
    }
}
