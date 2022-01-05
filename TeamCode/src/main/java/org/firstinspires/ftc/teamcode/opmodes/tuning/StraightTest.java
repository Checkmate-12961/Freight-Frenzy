package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;

/*
 * This is a simple routine to test translational drive capabilities.
 */

@SuppressWarnings("unused")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain robot = new Drivetrain(hardwareMap);

        Trajectory trajectory = robot.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.followTrajectoryAsync(trajectory);

        while (!isStopRequested() && opModeIsActive()){
            robot.update();

            Pose2d poseEstimate = robot.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (!robot.isBusy()){
                break;
            }
        }

        Pose2d poseEstimate = robot.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        robot.cleanup();
    }
}
