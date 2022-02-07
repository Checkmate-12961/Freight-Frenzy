package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Disabled
@Autonomous
class OpenCvTest : BaseOpMode() {
    override fun setupLoop() {
        telemetry.addData("Position", robot.barcode.position)
        telemetry.addData("Analysis", robot.barcode.analysis)
        telemetry.update()
    }
}