package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.robot.util.Pose2dConversionUtil;

public class RealsenseLocalizer implements Localizer {
    // TODO: tune this based on the relative positions of the camera and the robot
    public static Transform2d cameraRobotOffset = new Transform2d(new Translation2d(9.0,8.25), new Rotation2d());
    public double encoderMeasurementCovariance = 0.8;

    private final ElapsedTime elapsedTime;
    private double lastTime;
    private Pose2d lastPose = null;

    private final T265Camera slamera;


    public RealsenseLocalizer(HardwareMap hardwareMap) {
        super();
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        slamera = new T265Camera(cameraRobotOffset, encoderMeasurementCovariance, hardwareMap.appContext);
        slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d());
        slamera.start();
    }

    /**
     * Gets the localizer's estimated position
     * @return The position
     */
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return Pose2dConversionUtil.toRoadrunnerPose(slamera.getLastReceivedCameraUpdate().pose);
    }

    /**
     * Sets the localizer's estimated position
     * @param pose The position
     */
    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        slamera.setPose(Pose2dConversionUtil.toRealsensePose(pose));
    }

    /**
     * Gets the velocity between the current pose and the pose the last time this function was called
     * @return The velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        Pose2d poseVelo = null;
        Pose2d currentPose = getPoseEstimate();
        double currentTime = elapsedTime.time();

        // Only return a Pose2d if it won't cause a NPE
        if (lastPose != null) {
            poseVelo = currentPose.minus(lastPose).div(currentTime - lastTime);
        }

        // Updates what the "last" time and place are
        lastTime = currentTime;
        lastPose = currentPose;
        return poseVelo;
    }

    /**
     * Updates all of the nothing
     */
    @Override
    public void update() {
    }
}
