package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import kotlin.math.PI

@Autonomous
class BlueSide: BaseOpMode() {
    private val startPosition = Pose2d(-34.25, 62.0, PI /2)
    lateinit var sequence: TrajectorySequence

    override fun setup() {
        robot.drivetrain.poseEstimate = startPosition

        sequence = robot.drivetrain.trajectorySequenceBuilder(startPosition)
            .back(4.0)
            .turn(-PI /2)
            .lineToSplineHeading(Pose2d(-52.0, 56.0, 0.0))
            .addDisplacementMarker {
                robot.carousel.power = 1.0
                sleep(1500)
                robot.carousel.power = 0.0
            }
            .splineTo(Vector2d(-58.0, 36.0), -PI /2)
            .splineTo(Vector2d(-29.0, 25.0), 0.0)
            .addDisplacementMarker{
                robot.lift.target = robot.barcode.position.toLiftPoint()
                sleep(2000)
                robot.bucket.position = Bucket.Positions.DUMP
                sleep(1500)
                robot.lift.target = Lift.Points.MIN
            }
            .lineTo(Vector2d(-40.0, 25.0))
            .splineTo(Vector2d(-58.0, 36.0), PI /2)
            .build()
    }

    override fun preRunLoop() {
        robot.drivetrain.poseEstimate = startPosition
        robot.drivetrain.followTrajectorySequenceAsync(sequence)
    }
}