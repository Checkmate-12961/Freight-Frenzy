package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

/**
 * Subsystem to manage the capper mechanism.
 *
 * @constructor
 * Initialize the servos in the capper mechanism, set the direction, and set the position to [Positions.REST].
 *
 * @param hardwareMap Passed in from [org.firstinspires.ftc.teamcode.robot.CheckmateRobot].
 */
@Config
class Capper(hardwareMap: HardwareMap): AbstractSubsystem {
    override val tag = "Capper"
    override val subsystems = SubsystemMap{ tag }

    private var shoulderServo = Servos.CAP_SHOULDER.get(hardwareMap)
    private var elbowServo = Servos.CAP_ELBOW.get(hardwareMap)

    /**
     * Position of the capper mechanism.
     *
     * @see Positions
     */
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

    /**
     * Move the capper mechanism to the next position.
     */
    fun nextPosition() {
        position = position.next()
    }

    /**
     * Move the capper mechanism to the previous position.
     */
    fun prevPosition() {
        position = position.prev()
    }

    /**
     * Possible positions for the capper mechanism.
     */
    enum class Positions {
        REST, COLLECT, CAP;

        /**
         * Get the next position in order.
         *
         * @return The next position.
         */
        fun next(): Positions {
            return when(this) {
                REST -> COLLECT
                COLLECT -> CAP
                else -> this
            }
        }

        /**
         * Get the previous position in order.
         *
         * @return The previous position.
         */
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
        position = Positions.REST
    }
}