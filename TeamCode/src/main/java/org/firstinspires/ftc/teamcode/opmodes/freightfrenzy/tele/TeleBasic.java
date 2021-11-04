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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.abstracts.BasicOpMode;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

import java.util.Locale;
import java.util.Objects;

@TeleOp
public class TeleBasic extends BasicOpMode {
    @Override
    public void setup(){
        // Retrieve our pose from the PoseStorage.currentPose static field
        robot.drivetrain.setPoseEstimate(PositionUtil.get());

        // A opens the jank hand
        gp2.a.onPress = () -> robot.jankHand.close();
        // B closes the jank hand
        gp2.b.onPress = () -> robot.jankHand.open();

        // Right bumper runs the carousel
        gp2.rightBumper.onPress = () -> robot.carousel.setPower(1);
        gp2.rightBumper.onRelease = () -> robot.carousel.setPower(0);

        // Left bumper runs the carousel the other way
        gp2.leftBumper.onPress = () -> robot.carousel.setPower(-1);
        gp2.leftBumper.onRelease = () -> robot.carousel.setPower(0);
    }

    @Override
    public void run_loop(){
        updatePosition();

        switch (opModeType){
            case TeleOp:
                // Moves the robot based on the GP1 left stick
                runDrivetrain();

                // Runs the lift based on the GP2 dpad
                //runLift();
                runArm();
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
        Pose2d velocity = Objects.requireNonNull(robot.drivetrain.getPoseVelocity());
        PositionUtil.set(position);
        // Print pose to telemetry
        telemetry.addData("armAngle", Math.toDegrees(robot.jankArm.getAngle()));
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("h", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",getRuntime()));
        telemetry.addData("vX", velocity.getX());
        telemetry.addData("vY", velocity.getY());
        telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }

    // BIND:
    //  gp1.leftStickX, gp1.leftStickY
    //  gp1.rightStickX, gp1.rightTrigger
    private void runDrivetrain() {
        // Set the power of the DT based on the GP1 left
        robot.drivetrain.setWeightedDrivePower(
                new Pose2d(
                        // left stick X
                        -gp1.getLeftStickY() * Range.scale((gp1.getRightTrigger()), -1, 1, 0, 1),
                        // left sick Y
                        -gp1.getLeftStickX() * Range.scale((gp1.getRightTrigger()), -1, 1, 0, 1),
                        // right stick X (rotation)
                        -gp1.getRightStickX() * Range.scale((gp1.getRightTrigger()), -1, 1, 0, 1)
                )
        );
    }

    /*
    // BIND:
    //  gp2.dpad_up, gp2.dpad_down
    private void runLift() {
        if (gp2.dpad_up) {
            robot.lift.setHeight(robot.lift.getHeight() + .1);
        } else if (gp2.dpad_down) {
            robot.lift.setHeight(robot.lift.getHeight() - .1);
        }
    }
    */

    // BIND:
    //  gp2.dpadUp, gp2.dpadDown
    private void runArm() {
        if (gp2.getLeftStickY() < -0.4) {
            robot.jankArm.setAngle(robot.jankArm.getAngle() + .01);
        } else if (gp2.getLeftStickY() > 0.4) {
            robot.jankArm.setAngle(robot.jankArm.getAngle() - .01);
        }
    }
}
