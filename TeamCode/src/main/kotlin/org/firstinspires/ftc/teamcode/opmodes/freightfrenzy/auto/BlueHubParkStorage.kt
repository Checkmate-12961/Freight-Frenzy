package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.util.OpModeUtil.SuperPose2d
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Barcode
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Autonomous(preselectTeleOp = "TeleOp")
@Config
class BlueHubParkStorage: BaseOpMode() {
    private lateinit var sequence: TrajectorySequence

    companion object {
        // start
        @JvmField var pose0Start = SuperPose2d(-45.0, 62.0, 90.0)
        @JvmField var pose1ToHub = SuperPose2d(-12.5, 43.0, 90.0)
    }

    override fun setup() {
        robot.drivetrain.poseEstimate = pose0Start.pose2d

        sequence = robot.drivetrain.trajectorySequenceBuilder(pose0Start.pose2d)
            .setReversed(true)
            .lineToSplineHeading(pose1ToHub.pose2d)
            .addDisplacementMarker { // this is amazing! no idea why java doesn't do this
                sleep(5000)
                robot.lift.target = when(robot.barcode.position) {
                    Barcode.BarcodePosition.LEFT -> Lift.Points.LOW
                    Barcode.BarcodePosition.MIDDLE -> Lift.Points.MID
                    Barcode.BarcodePosition.RIGHT -> Lift.Points.HIGH
                }
                sleep(1000)
                robot.bucket.position = Bucket.Positions.DUMP
                sleep(2000)
                robot.bucket.position = Bucket.Positions.REST
                robot.lift.target = Lift.Points.MIN
            }
            .build()
    }

    override fun preRunLoop() {
        robot.drivetrain.followTrajectorySequenceAsync(sequence)
    }
}
