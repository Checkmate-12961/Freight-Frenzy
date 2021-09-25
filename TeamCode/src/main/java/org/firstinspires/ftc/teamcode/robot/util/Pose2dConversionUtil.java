package org.firstinspires.ftc.teamcode.robot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class Pose2dConversionUtil {
    /**
     * Converts a ftclib pose to a roadrunner pose
     * @param inputPose ftclib pose to convert
     * @return roadrunner pose with the same value as the input
     */
    public static Pose2d toRoadrunnerPose(com.arcrobotics.ftclib.geometry.Pose2d inputPose){
        return new Pose2d(inputPose.getX(), inputPose.getY(), inputPose.getHeading());
    }

    /**
     * Converts a roadrunner pose to a ftclib pose
     * @param inputPose roadrunner pose to convert
     * @return ftclib pose with the same value as the input
     */
    public static com.arcrobotics.ftclib.geometry.Pose2d toRealsensePose(Pose2d inputPose){
        return new com.arcrobotics.ftclib.geometry.Pose2d(
                inputPose.getX(),
                inputPose.getY(),
                new Rotation2d(inputPose.getHeading()));
    }
}
