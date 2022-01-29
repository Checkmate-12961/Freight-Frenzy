package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Autonomous
class park : BaseOpMode() {
    override fun setupLoop() {
        telemetry.addData("confidence", (robot.drivetrain.localizer as T265Localizer).poseConfidence)
    }

    lateinit var park: TrajectorySequence

    override fun setup() {
        park = robot.drivetrain.trajectorySequenceBuilder(Pose2d())
            .back(26.0)
            .strafeLeft(7.0)
            .build()
        robot.drivetrain.poseEstimate = park.start()
        robot.drivetrain.followTrajectorySequenceAsync(park)
    }
}