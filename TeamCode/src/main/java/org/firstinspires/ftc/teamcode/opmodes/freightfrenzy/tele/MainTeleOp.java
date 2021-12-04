/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

import java.util.Locale;

@Config
@TeleOp(name = "TeleOp")
public class MainTeleOp extends BaseOpMode {
    public static double liftChangeSpeed = 0.2;

    @Override
    public void setup(){
        // Retrieve our pose from the PoseStorage.currentPose static field
        robot.drivetrain.setPoseEstimate(PositionUtil.get());

        // Right bumper runs the carousel
        //gp2.rightBumper.onActivate = () -> robot.carousel.setPower(1);
        //gp2.rightBumper.onDeactivate = () -> robot.carousel.setPower(0);

        // Left bumper runs the carousel the other way
        //gp2.leftBumper.onActivate = () -> robot.carousel.setPower(-1);
        //gp2.leftBumper.onDeactivate = () -> robot.carousel.setPower(0);

        // Left stick Y axis runs the arm
        gp2.leftStickY.setActivationThreshold(0.4);
        gp2.leftStickY.whileActive = () -> robot.lift.setHeight(robot.lift.getHeight() - liftChangeSpeed);
        gp2.leftStickY.whileActiveNeg = () -> robot.lift.setHeight(robot.lift.getHeight() + liftChangeSpeed);

        // Dpad does set points
        gp2.dpadUp.onActivate = () -> robot.lift.setTarget(Lift.Points.HIGH);
        gp2.dpadRight.onActivate = () -> robot.lift.setTarget(Lift.Points.LOW);
        gp2.dpadLeft.onActivate = () -> robot.lift.setTarget(Lift.Points.LOW);
        gp2.dpadDown.onActivate = () -> robot.lift.setTarget(Lift.Points.MIN);

        // X wiggles the bucket
        gp2.x.onActivate = () -> robot.bucket.setPosition(Bucket.Positions.REST);
        gp2.x.onDeactivate = () -> robot.bucket.setPosition(Bucket.Positions.ZERO);

        // Right trigger dumps the bucket
        gp2.rightTrigger.setActivationThreshold(0.5);
        gp2.rightTrigger.onActivate = () -> robot.bucket.setPosition(Bucket.Positions.DUMP);
        gp2.rightTrigger.onDeactivate = () -> robot.bucket.setPosition(Bucket.Positions.REST);

        // A & B run the intake
        gp2.a.onActivate = () -> robot.intake.setPower(-1);
        gp2.a.onDeactivate = () -> robot.intake.setPower(0);
        gp2.b.onActivate = () -> robot.intake.setPower(1);
        gp2.b.onDeactivate = () -> robot.intake.setPower(0);

    }

    @Override
    public void run_loop(){
        updatePosition();

        switch (opModeType){
            case TeleOp:
                // Moves the robot based on the GP1 left stick
                robot.drivetrain.setWeightedDrivePower(
                        new Pose2d(
                                // left stick X
                                -gp1.leftStickY.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1),
                                // left sick Y
                                -gp1.leftStickX.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1),
                                // right stick X (rotation)
                                -gp1.rightStickX.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1)
                        )
                );
                break;

            case Autonomous:
                // Replace false here with a check to cancel the sequence
                //noinspection ConstantConditions
                if (false) robot.drivetrain.cancelSequence();
                if (!robot.drivetrain.isBusy()) opModeType = OpModeType.TeleOp;
                break;
            default:
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                opModeType = OpModeType.TeleOp;
                // Mission accomplished.
                break;
        }
    }

    private void updatePosition() {
        Pose2d position = robot.drivetrain.getPoseEstimate();
        Pose2d velocity = robot.drivetrain.getPoseVelocity();
        PositionUtil.set(position);
        // Print pose to telemetry
        telemetry.addData("liftHeight", robot.lift.getHeight());
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("h", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",getRuntime()));
        if (velocity != null) {
            telemetry.addData("vX", velocity.getX());
            telemetry.addData("vY", velocity.getY());
            telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        }
        telemetry.update();
    }
}
