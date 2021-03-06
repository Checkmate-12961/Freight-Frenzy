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
package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.abstracts.Triggerables.TriggerableCallback
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftPower

@Config
@TeleOp(name = "ManualTeleOp")
class ManualTeleOp : BaseOpMode() {
    private val CheckmateRobot.liftPower: LiftPower
        get() = subsystems["Lift"] as LiftPower

    override fun preSetup() {
        telemetry.addData("SHOULD YOU INIT", false)
        telemetry.update()
    }

    override fun setupLoop() {
        telemetry.addData("SHOULD YOU INIT",
            if (robot.barcode.isStreaming != null) robot.barcode.isStreaming
            else false
        )
        telemetry.update()
    }

    override fun setup() {
        robot.subsystems["Lift"] = LiftPower(hardwareMap, robot.bucket, robot.intake)

        // Right bumper runs the carousel
        gp2.rightBumper.onActivate = TriggerableCallback { robot.carousel.power = 1.0 }
        gp2.rightBumper.onDeactivate = TriggerableCallback { robot.carousel.power = 0.0 }

        // Left bumper runs the carousel the other way
        gp2.leftBumper.onActivate = TriggerableCallback { robot.carousel.power = -1.0 }
        gp2.leftBumper.onDeactivate = TriggerableCallback { robot.carousel.power = 0.0 }

        // Left trigger zeroes the bucket
        gp2.leftTrigger.onActivate = TriggerableCallback {
            robot.bucket.position = Bucket.Positions.ZERO
        }

        gp2.leftStickY.whileActive = TriggerableCallback {
            robot.liftPower.power = gp2.leftStickY.correctedValue.toDouble()
        }
        gp2.leftStickY.whileActiveNeg = gp2.leftStickY.whileActive

        gp2.leftStickY.onDeactivate = TriggerableCallback {
            robot.liftPower.power = 0.0
        }
        gp2.leftStickY.onDeactivateNeg = gp2.leftStickY.onDeactivate

        // X rests the bucket
        gp2.x.onActivate = TriggerableCallback { robot.bucket.position = Bucket.Positions.REST }

        // Right trigger dumps the bucket
        gp2.rightTrigger.activationThreshold = 0.5
        gp2.rightTrigger.onActivate =
            TriggerableCallback { robot.bucket.position = Bucket.Positions.DUMP }
        gp2.rightTrigger.onDeactivate =
            TriggerableCallback { robot.bucket.position = Bucket.Positions.REST }

        // A & B run the intake
        gp2.a.onActivate = TriggerableCallback {
            robot.bucket.position = Bucket.Positions.ZERO
            robot.intake.power = -1.0
        }
        gp2.a.onDeactivate = TriggerableCallback { robot.intake.power = 0.0 }
        gp2.b.onActivate = TriggerableCallback { robot.intake.power = 1.0 }
        gp2.b.onDeactivate = TriggerableCallback { robot.intake.power = 0.0 }
    }

    override fun runLoop() {
        when (opModeType) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.drivetrain.setWeightedDrivePower(
                    Pose2d( // left stick X
                        -gp1.leftStickY.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        ),  // left sick Y
                        -gp1.leftStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        ),  // right stick X (rotation)
                        -gp1.rightStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        )
                    )
                )
            OpModeType.Autonomous -> {
                // Replace false here with a check to cancel the sequence
                if (false) robot.drivetrain.cancelSequence()
                if (!robot.drivetrain.isBusy) opModeType = OpModeType.TeleOp
            }
            else ->
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                opModeType = OpModeType.TeleOp
        }
    }
}

