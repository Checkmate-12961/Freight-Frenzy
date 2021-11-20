package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous
class Schmoove : BaseOpMode() {
    override fun setup() {}
    override fun run_loop() {}

    override fun start() {
        robot.drivetrain.setWeightedDrivePower(Pose2d(1.0, 0.0, 0.0))
        try {
            Thread.sleep(1000)
        } catch (ignored: InterruptedException) {
        }
        robot.drivetrain.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
    }
}