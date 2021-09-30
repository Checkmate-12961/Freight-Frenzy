package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.CheckmateDrive;
import org.firstinspires.ftc.teamcode.robot.PoseUtils;

import java.util.Locale;

@TeleOp
public class TeleBase extends LinearOpMode {
    // Timer to keep track of elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    private enum ControlMode {TELE, AUTO}
    private ControlMode controlMode = ControlMode.TELE;

    // Define CheckmateDrive
    CheckmateDrive robot;

    @Override
    public void runOpMode() throws InterruptedException{
        // Initialize CheckmateDrive
        robot = new CheckmateDrive(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        robot.setPoseEstimate(PoseUtils.currentPose);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            controlRobot();
        }
        PoseUtils.currentPose = PoseUtils.getStartPose();
    }

    private void controlRobot() {
        // Update everything. Odometry. Etc.
        robot.update();
        updatePosition();

        switch (controlMode){
            case TELE:
                // Moves the robot based on the GP1 left stick
                runDrivetrain();
                break;

            case AUTO:
                // Replace false here with a check to cancel the sequence
                //noinspection ConstantConditions
                if (false) robot.cancelSequence();
                if (!robot.isBusy()) controlMode = ControlMode.TELE;
                break;
            default:
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                controlMode = ControlMode.TELE;
                // Mission accomplished.
                break;
        }
    }

    private void updatePosition() {
        Pose2d position = robot.getPoseEstimate();
        PoseUtils.currentPose = position;
        // Print pose to telemetry
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("heading", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",runtime.seconds()));
        telemetry.update();
    }

    // BIND:
    //  gamepad1.left_stick_x, gamepad1.left_stick_y
    //  gamepad1.right_stick_x, gamepad1.right_stick_y
    //  gamepad1.right_trigger
    private void runDrivetrain() {
        // Set the power of the DT based on the GP1 left
        robot.setWeightedDrivePower(
                new Pose2d(
                        // left stick X
                        gamepad1.left_stick_x * Range.scale((gamepad1.right_trigger), -1, 1, 0, 1),
                        // left sick Y
                        gamepad1.left_stick_y * Range.scale((gamepad1.right_trigger), -1, 1, 0, 1),
                        // right stick X (rotation)
                        -gamepad1.right_stick_x * Range.scale((gamepad1.right_trigger), -1, 1, 0, 1)
                )
        );
    }
}
