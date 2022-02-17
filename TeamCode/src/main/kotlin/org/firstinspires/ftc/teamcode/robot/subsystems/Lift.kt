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
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemContext
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import kotlin.math.PI

/**
 * Subsystem to manage the lift mechanism.
 *
 * @property bucket Instance of [Bucket] for failsafe purposes.
 * @property intake Instance of [Intake] for failsafe purposes.
 * @constructor
 * Initialize the motor, set the direction, mode, etc.
 *
 * @param context Passed in from [org.firstinspires.ftc.teamcode.robot.CheckmateRobot].
 */
@Config
class Lift(
    context: SubsystemContext
): AbstractSubsystem {
    override val tag = "Lift"
    override val subsystems = SubsystemMap{ tag }

    private val intake: Intake = context.subsystems["Intake"]!! as Intake
    private val bucket: Bucket = context.subsystems["Bucket"]!! as Bucket

    private val liftMotor = Motors.LIFT.get(context.hardwareMap)

    // Keep track of the last position the motor was set to
    private var lastPosition = 0.0

    private var offset = 0

    companion object {
        @JvmField var liftBounds = LiftBounds(0.0, 20.0)
        @JvmField var liftSetPoints = LiftSetPoints(12.0, 15.0, 27.0)
        @JvmField var runIntakeThreshold = 5.2
    }
    data class LiftBounds(@JvmField var min: Double, @JvmField var max: Double)
    data class LiftSetPoints(
        @JvmField var low: Double, @JvmField var mid: Double, @JvmField var high: Double)

    /**
     * Represents the possible positions of the lift.
     */
    enum class Points {
        LOW, MID, HIGH, MIN, MAX
    }

    // Multiplier for the height of the lift
    private val spoolDiameter = 1.5 // inches
    private val encoderTicksPerRev = 384.5
    private val ticksPerInch = encoderTicksPerRev / (spoolDiameter * PI)

    private var runIntakeLatch = false

    /**
     * Whether the lift is at the bottom and not making corrections.
     */
    var isDown: Boolean = true
        private set

    override fun loop() {
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

        isDown = !liftMotor.isBusy && tempHeight == 0.0
        if (isDown && bucket.position != Bucket.Positions.ZERO) {
            bucket.position = Bucket.Positions.ZERO
        } else if (!isDown && bucket.position == Bucket.Positions.ZERO) {
            bucket.position = Bucket.Positions.REST
        }
    }

    /**
     * The height of the lift in inches.
     */
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
                liftMotor.targetPosition = positionTicks + offset
            }
        }
        get() = liftMotor.targetPosition.toDouble() / ticksPerInch - offset

    /**
     * The target position of the lift.
     *
     * @see Points
     */
    var target: Points? = null
        set(value) {
            field = value
            when (value) {
                Points.MIN -> height = liftBounds.min
                Points.LOW -> height = liftSetPoints.low
                Points.MID -> height = liftSetPoints.mid
                Points.HIGH -> height = liftSetPoints.high
                Points.MAX -> height = liftBounds.max
                else -> {}
            }
        }

    init {
        // Initialize the motor
        liftMotor.power = 0.0
        liftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        offset = liftMotor.currentPosition

        // Set it to run to a target position and hold it
        liftMotor.targetPosition = offset
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.power = 1.0
    }
}