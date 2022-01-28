package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import kotlin.math.PI

@Autonomous
class BlueSide : BaseOpMode() {
    override fun setupLoop() {
        telemetry.addData("confidence", (robot.drivetrain.localizer as T265Localizer).poseConfidence)
    }
    private val startPose = Pose2d(-34.25, 62.0, PI /2)

    lateinit var trajectory: TrajectorySequence

    override fun setup() {
        trajectory = robot.drivetrain.trajectorySequenceBuilder(startPose)
            .back(4.0)
            .turn(-PI /2)
            .setReversed(true)
            .splineTo(Vector2d(-52.0, 56.0), PI)
            .addDisplacementMarker {
                robot.carousel.power = 1.0
                sleep(1500)
                robot.carousel.power = 0.0
            }
            .setReversed(false)
            .splineTo(Vector2d(-45.0, 54.0), 0.0)
            .turn(-PI * 3/4)
            .splineTo(Vector2d(-58.0, 36.0), -PI /2)
            .build()
    }

    override fun preRunLoop() {
        robot.drivetrain.poseEstimate = startPose
        robot.drivetrain.followTrajectorySequenceAsync(
            trajectory
        )
    }
}