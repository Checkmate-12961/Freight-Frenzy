package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap, private val bucket: Bucket) : AbstractSubsystem {
    private val liftMotor: DcMotorEx

    @Config
    object LiftBounds {
        // Values to manage the height of the lift
        @JvmField var maxHeight = 20.0
        @JvmField var minHeight = 0.0
    }
    @Config
    object LiftSetPoints {
        @JvmField var low = 12.0
        @JvmField var mid = 15.0
        @JvmField var high = 20.0
    }

    enum class Points {
        LOW, MID, HIGH, MIN, MAX
    }

    // Multiplier for the height of the lift
    private val spoolDiameter = 1.5 // inches
    private val encoderTicksPerRev = 537.7
    private var ticksPerInch = encoderTicksPerRev / (spoolDiameter * PI)

    var height: Double
        set(value) {
            val temp = (
                    Range.clip(
                        value, LiftBounds.minHeight, LiftBounds.maxHeight
                    ) * ticksPerInch).toInt()
            if (temp == 0) {
                //bucket.position = Bucket.Positions.ZERO
            } else {
                bucket.position = Bucket.Positions.REST
            }
            liftMotor.targetPosition = temp
        }
        get() {
            return liftMotor.targetPosition.toDouble() / ticksPerInch
        }

    var target: Points? = null
        set(value) {
            field = value
            when (value) {
                Points.MIN -> height = LiftBounds.minHeight
                Points.LOW -> height = LiftSetPoints.low
                Points.MID -> height = LiftSetPoints.mid
                Points.HIGH -> height = LiftSetPoints.high
                Points.MAX -> height = LiftBounds.maxHeight
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
}