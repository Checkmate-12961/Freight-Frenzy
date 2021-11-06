package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;

/**
 * This is a simple routine to test turning capabilities.
 */
@Disabled
@SuppressWarnings("unused")
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        CheckmateRobot robot = new CheckmateRobot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        robot.drivetrain.turn(Math.toRadians(ANGLE));

        robot.cleanup();
    }
}
