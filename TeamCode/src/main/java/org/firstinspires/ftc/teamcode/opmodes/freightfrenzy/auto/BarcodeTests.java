package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;


@Autonomous(preselectTeleOp = "TeleBasic")
public class BarcodeTests extends BaseOpMode {
    /**
     * Main OpMode loop, automatically updates the robot
     */
    @Override
    public void runLoop() {
        telemetry.addData("barcodePosition", robot.getBarcode().getPosition());
        telemetry.update();
    }
}
