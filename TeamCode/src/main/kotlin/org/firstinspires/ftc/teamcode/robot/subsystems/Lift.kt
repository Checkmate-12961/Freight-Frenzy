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

@Config
class Lift(hardwareMap: HardwareMap, private val bucket: Bucket, private val intake: Intake) : AbstractSubsystem {
    private val liftMotor = hardwareMap.get(DcMotorEx::class.java, Motors.LIFT.id)

    // Keep track of the last position the motor was set to
    private var lastPosition = 0.0

    // This is weird because of the dashboard
    companion object {
        @JvmField var liftBounds = LiftBounds(0.0, 20.0)
        @JvmField var liftSetPoints = LiftSetPoints(12.0, 15.0, 20.0)
        @JvmField var runIntakeThreshold = 5.2
    }
    data class LiftBounds(@JvmField var min: Double, @JvmField var max: Double)
    data class LiftSetPoints(
        @JvmField var low: Double, @JvmField var mid: Double, @JvmField var high: Double)

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

    var isDown: Boolean = true
        private set

    override fun update() {
        val tempHeight = height

        if (!liftMotor.isBusy) {
            if (runIntakeLatch) {
                runIntakeLatch = false
                // When the lift stops moving, stop the intake
                intake.power = 0.0
            }
        } else {
            if (tempHeight < lastPosition && tempHeight < runIntakeThreshold
                    && !runIntakeLatch) {
                runIntakeLatch = true
                // When the lift starts moving down below the threshold, run the intake
                intake.power = .5
            }
        }

        if (!liftMotor.isBusy && tempHeight == 0.0) {
            isDown = true
            if (!zeroPositionLatch) {
                zeroPositionLatch = true
                // When the lift stops moving at the 0 point, set the set the bucket position
                //  to ZERO
                bucket.position = Bucket.Positions.ZERO
            }
        } else {
            isDown = false
            zeroPositionLatch = false
        }
    }

    var height: Double
        set(value) {
            // If this value is different from the last
            if (height != value) {
                lastPosition = height
                // Calculate the ticks
                val positionTicks = (
                        Range.clip(
                            value, liftBounds.min, liftBounds.max
                        ) * ticksPerInch).toInt()
                bucket.position = Bucket.Positions.REST
                liftMotor.targetPosition = positionTicks
            }
        }
        get() = liftMotor.targetPosition.toDouble() / ticksPerInch

    var target: Points? = null
        set(value) {
            field = value
            when (value) {
                Points.MIN -> height = liftBounds.min
                Points.LOW -> height = liftSetPoints.low
                Points.MID -> height = liftSetPoints.mid
                Points.HIGH -> height = liftSetPoints.high
                Points.MAX -> height = liftBounds.max
            }
        }

    init {
        // Initialize the motor
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Reverse the motor if the config says to
        if (Motors.LIFT.reverse) {
            liftMotor.direction = DcMotorSimple.Direction.REVERSE
        }

        // Set it to run to a target position and hold it
        liftMotor.targetPosition = 0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.power = 1.0
    }
}