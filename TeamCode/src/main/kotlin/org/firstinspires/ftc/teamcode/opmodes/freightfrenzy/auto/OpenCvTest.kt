package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous
class OpenCvTest : BaseOpMode() {
    override fun setupLoop() {
        telemetry.addData("Position", robot.barcode.position)
        telemetry.addData("Analysis", robot.barcode.analysis)
        telemetry.update()
    }
}