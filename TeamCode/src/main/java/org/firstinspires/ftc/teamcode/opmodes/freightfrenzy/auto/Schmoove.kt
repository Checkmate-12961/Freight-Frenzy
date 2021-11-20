package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous
class Schmoove : BaseOpMode() {
    override fun setup() {}
    override fun run_loop() {}

    /**
     * when you you you you when you run button
     */
    override fun start() {
        // go
        robot.drivetrain.setWeightedDrivePower(Pose2d(1.0, 0.0, 0.0))

        // wait
        try {
            Thread.sleep(1000)
        } catch (ignored: InterruptedException) {}

        // stop going
        robot.drivetrain.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
    }
}