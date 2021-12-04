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
        DUMP(0.42),
        REST(0.21),
        ZERO(0.17)
    }

    var position: Positions = Positions.REST
        set(value) {
            field = value
            bucketServo.position = value.position
        }

    init {
        bucketServo = hardwareMap.get(Servo::class.java, Servos.BUCKET.name)
        if (Servos.BUCKET.reversed) {
            bucketServo.direction = Servo.Direction.REVERSE
        }
        bucketServo.position = Positions.REST.position
    }
}