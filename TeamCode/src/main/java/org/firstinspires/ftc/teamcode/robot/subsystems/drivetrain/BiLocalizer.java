package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.bilocalizer.RealsenseLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.bilocalizer.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.util.PoseUtil;

import java.util.List;

/**
 * Localizer using both the tracking wheels and the realsense camera
 */
public class BiLocalizer implements Localizer {
    private final RealsenseLocalizer realsenseLocalizer;
    private final TrackingWheelLocalizer trackingWheelLocalizer;

    /**
     * Initializes both localizers
     * @param hardwareMap Hardware map passed in from an op mode
     */
    public BiLocalizer(HardwareMap hardwareMap) {
        super();

        realsenseLocalizer = new RealsenseLocalizer(hardwareMap);
        trackingWheelLocalizer = new TrackingWheelLocalizer(hardwareMap);
    }

    /**
     * Gets the most confident localizer's estimated position
     * @return The position
     */
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        T265Camera.CameraUpdate update = realsenseLocalizer.getRawUpdate();

        // TODO: get the bounding box thing working
        switch (update.confidence){
            case Failed:
            case Low:
            case Medium:
                return trackingWheelLocalizer.getPoseEstimate();
            default:
                return updateToPose(update);
        }
    }

    /**
     * Sets both localizers' estimated positions
     * @param pose2d The position
     */
    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        trackingWheelLocalizer.setPoseEstimate(pose2d);
        realsenseLocalizer.setPoseEstimate(pose2d);
    }

    /**
     * Gets the velocity of the current pose
     * @return The velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        // Using the camera's pose velocity because it takes the
        // tracking wheels into consideration when generating it
        return realsenseLocalizer.getPoseVelocity();
    }

    /**
     * Applies the tracking wheel velocity to the realsense camera and the updates
     *  the least confident localizer with the most confident one
     */
    @Override
    public void update() {
        List<Double> wheelVelocities = trackingWheelLocalizer.getWheelVelocities();
        realsenseLocalizer.sendOdometry(
                PoseUtil.inchesToMeters(
                    new Pose2d(wheelVelocities.get(0), wheelVelocities.get(2))
            )
        );

        // Get an update from the camera
        T265Camera.CameraUpdate update = realsenseLocalizer.getRawUpdate();

        // Get poses from both of our localizers
        Pose2d trackingWheelPose = trackingWheelLocalizer.getPoseEstimate();
        Pose2d realsensePose = PoseUtil.metersToInches(
                PoseUtil.toRoadrunnerPose(
                        update.pose));

        // Update the pose of the least confident localizer
        // TODO: get the bounding box thing working
        switch (update.confidence) {
            case Failed:
            case Low:
            case Medium:
                realsenseLocalizer.setPoseEstimate(trackingWheelPose);
                break;
            default:
                trackingWheelLocalizer.setPoseEstimate(realsensePose);
        }
    }

    /**
     * Pulls the pose from a T265 camera update and converts it to be used with RR
     * @param update Camera update
     * @return Pose2d (in inches)
     */
    private Pose2d updateToPose(T265Camera.CameraUpdate update) {
        return PoseUtil.metersToInches(
                PoseUtil.toRoadrunnerPose(
                        update.pose));
    }
}
