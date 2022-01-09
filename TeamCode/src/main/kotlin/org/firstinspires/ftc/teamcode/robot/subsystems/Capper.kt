package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

@Config
class Capper(hardwareMap: HardwareMap): AbstractSubsystem {
    private var shoulderServo = hardwareMap.get(Servo::class.java, Servos.CAP_SHOULDER.id)
    private var elbowServo = hardwareMap.get(Servo::class.java, Servos.CAP_ELBOW.id)

    var position = Positions.REST
        set(value) {
            field = value
            elbowServo.position = when(value) {
                Positions.CAP -> elbowPositions.cap
                Positions.COLLECT -> elbowPositions.collect
                Positions.REST -> elbowPositions.rest
            }
            shoulderServo.position = when(value) {
                Positions.CAP -> shoulderPositions.cap
                Positions.COLLECT -> shoulderPositions.collect
                Positions.REST -> shoulderPositions.rest
            }
        }

    fun nextPosition() {
        position = position.next()
    }

    fun prevPosition() {
        position = position.prev()
    }

    enum class Positions {
        REST, COLLECT, CAP;

        fun next(): Positions {
            return when(this) {
                REST -> COLLECT
                COLLECT -> CAP
                else -> this
            }
        }

        fun prev(): Positions {
            return when(this) {
                CAP -> COLLECT
                COLLECT -> REST
                else -> this
            }
        }
    }

    data class CapperServoBounds(
        @JvmField var cap: Double,
        @JvmField var collect: Double,
        @JvmField var rest: Double
    )

    companion object {
        @JvmField var elbowPositions = CapperServoBounds(
            .5,.5,.5
        )
        @JvmField var shoulderPositions = CapperServoBounds(
            .3,.3,.3
        )
    }

    init {
        if (Servos.CAP_ELBOW.reversed) {
            elbowServo.direction = Servo.Direction.REVERSE
        }
        if (Servos.CAP_SHOULDER.reversed) {
            shoulderServo.direction = Servo.Direction.REVERSE
        }

        position = Positions.REST
    }
}