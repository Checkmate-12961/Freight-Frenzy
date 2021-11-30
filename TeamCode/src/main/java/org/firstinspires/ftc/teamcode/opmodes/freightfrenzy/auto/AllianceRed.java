package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PoseUtil;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

import java.util.Locale;
import java.util.Objects;

@Disabled
@Autonomous(preselectTeleOp = "TeleBasic")
public class AllianceRed extends BaseOpMode {
    Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(90));

    @Override
    public void pre_setup() {
        PositionUtil.set(startPose);
    }

    /**
     * Runs when the OpMode initializes
     */
    @Override
    public void setup() {
        // Crash the op mode if the realsense camera doesn't init
        while (!PoseUtil.greaterThan(PoseUtil.abs(robot.drivetrain.getPoseEstimate()), 1, 1)) {
            robot.drivetrain.setPoseEstimate(startPose);
        }

        // Trajectory to get the robot into the shared thing on blue
        TrajectorySequence toSharedRed = robot.drivetrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-50, -50), Math.toRadians(135))
                .splineTo(new Vector2d(-59, -35), Math.toRadians(90))
                .build();

        robot.drivetrain.followTrajectorySequenceAsync(toSharedRed);
    }

    /**
     * Main OpMode loop, automatically updates the robot
     */
    @Override
    public void run_loop() {
        Pose2d position = robot.drivetrain.getPoseEstimate();
        Pose2d velocity = Objects.requireNonNull(robot.drivetrain.getPoseVelocity());
        PositionUtil.set(position);
        // Print pose to telemetry
        telemetry.addData("armAngle", Math.toDegrees(robot.lift.getHeight()));
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("h", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",getRuntime()));
        telemetry.addData("vX", velocity.getX());
        telemetry.addData("vY", velocity.getY());
        telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }
}
