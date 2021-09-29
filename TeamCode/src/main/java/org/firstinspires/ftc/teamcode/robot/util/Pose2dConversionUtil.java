package org.firstinspires.ftc.teamcode.robot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

/**
 * Class for converting Pose2d objects to be used with the Realsense camera library
 */
public class Pose2dConversionUtil {
    private static final double metersToInchesMultiplier = 100.0/2.54;

    /**
     * Converts a FTCLib pose to a Roadrunner pose
     * @param inputPose FTCLib pose to convert
     * @return Roadrunner pose with the same value as the input
     */
    public static Pose2d toRoadrunnerPose(com.arcrobotics.ftclib.geometry.Pose2d inputPose){
        return new Pose2d(inputPose.getX(), inputPose.getY(), inputPose.getHeading());
    }

    /**
     * Converts a Roadrunner pose to a FTCLib pose
     * @param inputPose Roadrunner pose to convert
     * @return FTCLib pose with the same value as the input
     */
    public static com.arcrobotics.ftclib.geometry.Pose2d toFtclibPose(Pose2d inputPose){
        return new com.arcrobotics.ftclib.geometry.Pose2d(
                inputPose.getX(),
                inputPose.getY(),
                new Rotation2d(inputPose.getHeading()));
    }

    /**
     * Converts measurements from meters to inches
     * @param inputPose pose to convert
     * @return converted pose
     */
    public static Pose2d metersToInches(Pose2d inputPose) {
        return new Pose2d(
                inputPose.vec().times(metersToInchesMultiplier),
                inputPose.getHeading());
    }

    /**
     * Converts measurements from inches to meters
     * @param inputPose pose to convert
     * @return converted pose
     */
    public static Pose2d inchesToMeters(Pose2d inputPose) {
        return new Pose2d(
                inputPose.vec().div(metersToInchesMultiplier),
                inputPose.getHeading());
    }
}
