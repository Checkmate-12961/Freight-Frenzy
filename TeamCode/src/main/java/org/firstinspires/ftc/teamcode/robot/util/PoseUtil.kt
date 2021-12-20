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
package org.firstinspires.ftc.teamcode.robot.util

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import kotlin.math.abs

/**
 * Class for converting Pose2d objects to be used with the Realsense camera library
 */
object PoseUtil {
    private const val metersToInchesMultiplier = 100.0 / 2.54

    /**
     * Converts a FTCLib pose to a Roadrunner pose
     * @param inputPose FTCLib pose to convert
     * @return Roadrunner pose with the same value as the input
     */
    @JvmStatic fun toRoadrunnerPose(inputPose: Pose2d): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            inputPose.x,
            inputPose.y,
            inputPose.heading
        )
    }

    /**
     * Converts a Roadrunner pose to a FTCLib pose
     * @param inputPose Roadrunner pose to convert
     * @return FTCLib pose with the same value as the input
     */
    @JvmStatic fun toFtclibPose(inputPose: com.acmerobotics.roadrunner.geometry.Pose2d): Pose2d {
        return Pose2d(
            inputPose.x,
            inputPose.y,
            Rotation2d(inputPose.heading)
        )
    }

    /**
     * Converts measurements from meters to inches
     * @param inputPose pose to convert
     * @return converted pose
     */
    @JvmStatic fun metersToInches(inputPose: com.acmerobotics.roadrunner.geometry.Pose2d): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            inputPose.vec().times(metersToInchesMultiplier),
            inputPose.heading
        )
    }

    /**
     * Converts measurements from inches to meters
     * @param inputPose pose to convert
     * @return converted pose
     */
    @JvmStatic fun inchesToMeters(inputPose: com.acmerobotics.roadrunner.geometry.Pose2d): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            inputPose.vec().div(metersToInchesMultiplier),
            inputPose.heading
        )
    }

    /**
     * Converts a ChassisSpeeds object to a Roadrunner Pose2d
     * @param chassisSpeeds ChassisSpeeds object to convert
     * @return Roadrunner pose with the same value
     */
    @JvmStatic fun chassisSpeedsToRoadrunnerPose(chassisSpeeds: ChassisSpeeds): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond
        )
    }

    /**
     * Converts a Roadrunner Pose2d to a FTCLib Transform2d
     * @param inputPose Pose to convert
     * @return Transform2d with the same value
     */
    @JvmStatic fun toTransform2d(inputPose: com.acmerobotics.roadrunner.geometry.Pose2d): Transform2d {
        return Transform2d(
            Translation2d(inputPose.x, inputPose.y),
            Rotation2d(inputPose.heading)
        )
    }

    /**
     * Multiply each individual component
     * @param inputPose Pose to multiply
     * @param xFactor Factor of the x component
     * @param yFactor Factor of the y component
     * @param headingFactor Factor of the heading component
     * @return Multiplied pose
     */
    @JvmStatic fun multiply(
        inputPose: com.acmerobotics.roadrunner.geometry.Pose2d,
        xFactor: Double,
        yFactor: Double,
        headingFactor: Double
    ): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            inputPose.x * xFactor,
            inputPose.y * yFactor,
            inputPose.heading * headingFactor
        )
    }

    /**
     * Check whether each of the position values is greater than another value
     * @param inputPose Pose to check
     * @param x X "other" value
     * @param y Y "other" value
     * @return Whether each component is greater than its "other" value
     */
    @JvmStatic fun greaterThan(
        inputPose: com.acmerobotics.roadrunner.geometry.Pose2d,
        x: Double,
        y: Double
    ): Boolean {
        return inputPose.x > x && inputPose.y > y
    }

    /**
     * Get the absolute value of each individual component
     * @param inputPose Pose to get the absolute value of
     * @return Absolute valued pose
     */
    @JvmStatic fun abs(inputPose: com.acmerobotics.roadrunner.geometry.Pose2d): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            abs(inputPose.x),
            abs(inputPose.y),
            inputPose.heading
        )
    }
}