package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

/**
 *  I'd just like to interject for a moment. What you're referring to as T265, is in fact,
 *  Square/T265, or as I've recently taken to calling it, Square plus T265. T265 is not an op mode
 *  unto itself, but rather another free component of a fully functioning Square system made useful
 *  by the T265 localization, velocity statistics and vital system components comprising a full op
 *  mode as defined by FIRST.
 *
 *  Basically, it uses T265 to go in a square.
 */
@Config
@Autonomous(name = "Square/T265", group = "test")
class SquarePlusT265: BaseOpMode() {
    private lateinit var squareTrajectory: TrajectorySequence
    private var doThing = false

    override fun setup() {
        squareTrajectory = robot.drivetrain.trajectorySequenceBuilder(Pose2d())
            .forward(squareLength)
            .build()
    }

    override fun setupLoop() {
        val packet = TelemetryPacket()
        val poseEstimate = robot.drivetrain.poseEstimate
        packet.put("x", poseEstimate.x)
        packet.put("y", poseEstimate.y)
        packet.put("heading", poseEstimate.heading * 180.0 / kotlin.math.PI)
        packet.put("poseConfidence", (robot.drivetrain.localizer as T265Localizer).poseConfidence)
        dash.sendTelemetryPacket(packet)
    }

    override fun preRunLoop() {
        robot.drivetrain.poseEstimate = Pose2d()
    }

    override fun runLoop() {}

    companion object {
        @JvmField var squareLength = 12.0
    }
}