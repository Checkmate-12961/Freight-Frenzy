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
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap, private val bucket: Bucket, private val intake: Intake) : AbstractSubsystem {
    private val liftMotor: DcMotorEx

    // Keep track of the last position the motor was set to
    private var lastPosition = 0.0

    // This is weird because of the dashboard
    @Config
    object Lift {
        @JvmField var liftBounds = LiftBounds(0.0, 20.0)
        @JvmField var liftSetPoints = LiftSetPoints(12.0, 15.0, 20.0)
        @JvmField var runIntakeThreshold = 5.2
    }

    enum class Points {
        LOW, MID, HIGH, MIN, MAX
    }

    // Multiplier for the height of the lift
    private val spoolDiameter = 1.5 // inches
    private val encoderTicksPerRev = 537.7
    private val ticksPerInch = encoderTicksPerRev / (spoolDiameter * PI)

    // Latches to prevent calling functions multiple times per triggering event
    private var zeroPositionLatch = false
    private var runIntakeLatch = false

    override fun update() {
        val tempHeight = height

        if (!liftMotor.isBusy) {
            if (runIntakeLatch) {
                runIntakeLatch = false
                intake.power = 0.0
            }
        } else {
            if (tempHeight < lastPosition && tempHeight < Lift.runIntakeThreshold && !runIntakeLatch) {
                runIntakeLatch = true
                intake.power = .5
            }
        }

        if (!liftMotor.isBusy && tempHeight == 0.0) {
            if (!zeroPositionLatch) {
                zeroPositionLatch = true
                bucket.position = Bucket.Positions.ZERO
            }
        } else {
            zeroPositionLatch = false
        }
    }

    var height: Double
        set(value) {
            lastPosition = height
            val temp = (
                    Range.clip(
                        value, Lift.liftBounds.min, Lift.liftBounds.max
                    ) * ticksPerInch).toInt()
            if (temp != 0) {
                bucket.position = Bucket.Positions.REST
            }
            liftMotor.targetPosition = temp
        }
        get() = liftMotor.targetPosition.toDouble() / ticksPerInch

    var target: Points? = null
        set(value) {
            field = value
            when (value) {
                Points.MIN -> height = Lift.liftBounds.min
                Points.LOW -> height = Lift.liftSetPoints.low
                Points.MID -> height = Lift.liftSetPoints.mid
                Points.HIGH -> height = Lift.liftSetPoints.high
                Points.MAX -> height = Lift.liftBounds.max
            }
        }

    init {
        // Initialize the motor
        liftMotor = hardwareMap.get(DcMotorEx::class.java, Motors.LIFT.name)
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Reverse the motor if the config says to
        if (Motors.LIFT.reverse) {
            liftMotor.direction = DcMotorSimple.Direction.REVERSE
        }

        // Set it to run to a target position and hold it
        height = 0.0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.power = 1.0
    }

    data class LiftBounds(@JvmField var min: Double, @JvmField var max: Double)
    data class LiftSetPoints(
        @JvmField var low: Double, @JvmField var mid: Double, @JvmField var high: Double)
}