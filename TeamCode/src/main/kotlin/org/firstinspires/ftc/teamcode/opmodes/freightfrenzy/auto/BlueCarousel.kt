package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.util.OpModeUtil.SuperPose2d
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Autonomous
@Config
class BlueCarousel: BaseOpMode() {
    private lateinit var sequence: TrajectorySequence

    companion object {
        // start
        @JvmField var pose0Start = SuperPose2d(-38.0, 56.0, -90.0)
        // reverse
        // go forward and spin the right way
        @JvmField var pose1ReOrient = SuperPose2d(-38.0, 54.0, 0.0)
        // back up to the carousel
        @JvmField var pose2ToCarousel = SuperPose2d(-56.0, 54.0, 0.0)
        // spin 
        // un-reverse
        // go to box
        @JvmField var pose3ToBox = SuperPose2d(-61.0, 38.0, 90.0)
    }

    override fun setup() {
        robot.drivetrain.poseEstimate = pose0Start.pose2d

        sequence = robot.drivetrain.trajectorySequenceBuilder(pose0Start.pose2d)
            .setReversed(true)
            .splineTo(pose1ReOrient.vector2d, pose1ReOrient.hRad)
            .splineTo(pose2ToCarousel.vector2d, pose2ToCarousel.hRad)
            .addDisplacementMarker { // this is amazing! no idea why java doesn't do this
                robot.carousel.power = 1.0
                sleep(1000)
                robot.carousel.power = 0.0
            }
            .setReversed(false)
            .splineTo(pose3ToBox.vector2d, pose3ToBox.hRad)
            .build()
    }

    override fun preRunLoop() {
        robot.drivetrain.followTrajectorySequenceAsync(sequence)
    }
}
