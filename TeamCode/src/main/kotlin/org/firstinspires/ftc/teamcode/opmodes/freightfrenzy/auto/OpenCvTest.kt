package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous(preselectTeleOp = "TeleBasic")
class OpenCvTest : BaseOpMode() {
    /**
     * Main OpMode loop, automatically updates the robot
     */
    override fun runLoop() {
        telemetry.addData("Position", robot.barcode.position)
        telemetry.addData("Analysis", robot.barcode.analysis)
        telemetry.update()
    }
}