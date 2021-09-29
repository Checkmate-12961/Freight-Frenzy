package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.robot.util.Pose2dConversionUtil;

/**
 * Localizer that implements Intel's T265 Realsense camera's VSLAM system
 */
public class RealsenseLocalizer implements Localizer {
    // TODO: tune this based on the relative positions of the camera and the robot
    //  This transform is the distance between the camera and the robot (not vice versa)
    //  Use the normal coordinate system the field uses.
    public static Transform2d cameraRobotOffset = Pose2dConversionUtil.toTransform2d(
            Pose2dConversionUtil.inchesToMeters(
                    new Pose2d(-9.0, -8.25)));
    public double encoderMeasurementCovariance = 0.8;

    private final T265Camera slamera;

    private T265Camera.PoseConfidence lastConfidence = null;

    /**
     * Initializes the localizer and starts receiving packets from it
     * @param hardwareMap Passed in from the OpMode
     */
    public RealsenseLocalizer(HardwareMap hardwareMap) {
        super();
        slamera = new T265Camera(cameraRobotOffset, encoderMeasurementCovariance, hardwareMap.appContext);
        slamera.stop();
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
        T265Camera.CameraUpdate update = slamera.getLastReceivedCameraUpdate();
        lastConfidence = update.confidence;

        // Convert meters to inches
        return Pose2dConversionUtil.metersToInches(
                // Convert to Roadrunner Pose2d
                Pose2dConversionUtil.toRoadrunnerPose(
                        update.pose));
    }

    /**
     * Sets the localizer's estimated position
     * @param pose The position
     */
    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        lastConfidence = T265Camera.PoseConfidence.High;
        slamera.setPose(
                // Convert to FTCLib Pose2d
                Pose2dConversionUtil.toFtclibPose(
                        // Convert inches to meters
                        Pose2dConversionUtil.inchesToMeters(
                                pose)));
    }

    /**
     * Gets the velocity between the current pose and the pose the last time this function was called
     * @return The velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        // Convert meters to inches
        return Pose2dConversionUtil.metersToInches(
                // Convert ChassisSpeeds to Pose2d
                Pose2dConversionUtil.chassisSpeedsToRoadrunnerPose(
                        // Get the camera's reported velocity
                        slamera.getLastReceivedCameraUpdate().velocity));
    }

    /**
     * Updates all of the nothing
     */
    @Override
    public void update() {
    }

    /**
     * Gets the T265's confidence level on the latest getPoseEstimate call
     * DO NOT RELY ON THE ACCURACY OF THIS!
     * @return Confidence level
     */
    public T265Camera.PoseConfidence getPoseConfidence() {
        return lastConfidence;
    }

    /**
     * Gets the last update received from the camera
     * @return The CameraUpdate
     */
    public T265Camera.CameraUpdate getRawUpdate() {
        return slamera.getLastReceivedCameraUpdate();
    }
}
