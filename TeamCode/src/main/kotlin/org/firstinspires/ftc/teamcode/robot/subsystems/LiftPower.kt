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

package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

@Config
class LiftPower(
    hardwareMap: HardwareMap, private val bucket: Bucket, private val intake: Intake
): AbstractSubsystem {
    override val tag = "LiftPower"
    override val subsystems = SubsystemMap{ tag }

    private val liftMotor = hardwareMap.get(DcMotorEx::class.java, Motors.LIFT.id)

    // This is weird because of the dashboard
    companion object {
        @JvmField var powerCoefficient = 0.7
    }

    var power: Double
        set(value) {
            //bucket.position = Bucket.Positions.REST
            if (-value < 0.0) {
                intake.power = 1.0
            } else {
                if (bucket.position == Bucket.Positions.ZERO) {
                    bucket.position = Bucket.Positions.REST
                }
                intake.power = 0.0
            }
            liftMotor.power = Range.clip(-value, -1.0, 1.0) * powerCoefficient
        }
        get() = liftMotor.power

    init {
        // Initialize the motor
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // Reverse the motor if the config says to
        if (Motors.LIFT.reverse) {
            liftMotor.direction = DcMotorSimple.Direction.REVERSE
        }

        liftMotor.power = 0.0
    }
}