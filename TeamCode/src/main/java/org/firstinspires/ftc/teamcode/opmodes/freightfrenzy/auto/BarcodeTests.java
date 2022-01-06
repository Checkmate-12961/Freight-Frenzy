package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Barcode;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PoseUtil;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

import java.util.Locale;
import java.util.Objects;


@Autonomous(preselectTeleOp = "TeleBasic")
public class BarcodeTests extends BaseOpMode {

    @Override
    public void pre_setup() {

    }

    /**
     * Runs when the OpMode initializes
     */
    @Override
    public void setup() {
        Barcode barcode = new Barcode(hardwareMap);
        Barcode.BarcodePosition barcodePosition;
        barcodePosition = barcode.getPosition();
        telemetry.addData("barcodePosition", barcodePosition);
    }

    /**
     * Main OpMode loop, automatically updates the robot
     */
    @Override
    public void run_loop() {
        /*Barcode barcode = new Barcode(hardwareMap);
        Barcode.BarcodePosition barcodePosition;
        barcodePosition = barcode.getPosition();
        telemetry.addData("barcodePosition", barcodePosition);*/
    }
}
