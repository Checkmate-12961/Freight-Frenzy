package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode

@Autonomous(preselectTeleOp = "TeleBasic")
class OpenCvTest : BaseOpMode() {
    /**
     * Runs when the OpMode initializes
     */
    override fun setup() {
        // nothing to set up
    }

    /**
     * Main OpMode loop, automatically updates the robot
     */
    override fun run_loop() {
        telemetry.addData("Position", robot.barcode.position)
        telemetry.addData("Analysis", robot.barcode.analysis)
        telemetry.update()
    }
}