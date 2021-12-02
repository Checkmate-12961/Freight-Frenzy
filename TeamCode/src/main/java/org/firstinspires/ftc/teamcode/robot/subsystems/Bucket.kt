package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

/**
 * Bucket thingy on the lift. There is no direct access to the servo position for a reason.
 */
class Bucket(hardwareMap: HardwareMap) : AbstractSubsystem {
    private var bucketServo: Servo

    /**
     * Set positions for the bucket.
     *
     * All of these values are derived from dinking around.
     *  DO NOT TOUCH THEM OR SIMON WILL TAKE YOUR KNEECAPS.
     */
    enum class Positions(@JvmField val position: Double) {
        // Position when the bucket is dumping
        DUMP(0.42),
        // The normal position
        REST(0.21),
        // The position when the bucket is all the way down
        ZERO(0.17)
    }

    /**
     * The position the bucket is at
     *
     * These aren't raw double values, they're values on the enum so that programmers don't break
     *  things
     */
    var position: Positions = Positions.REST
        set(value) {
            field = value
            bucketServo.position = value.position
        }

    init {
        // Grab the servo from the hardware map
        bucketServo = hardwareMap.get(Servo::class.java, Servos.BUCKET.name)

        // If the servo is reversed in the config, reverse it
        if (Servos.BUCKET.reversed) {
            bucketServo.direction = Servo.Direction.REVERSE
        }

        // Set the position to the rest position
        bucketServo.position = Positions.REST.position
    }
}