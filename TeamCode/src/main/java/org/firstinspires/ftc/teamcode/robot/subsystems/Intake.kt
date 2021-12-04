package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

class Intake(hardwareMap: HardwareMap) : AbstractSubsystem {
    private val intakeMotor: DcMotorEx

    @Config
    object IntakePower{
        @JvmField var coefficient = .7
    }

    var power: Double
        /**
         * Positive values push stuff out, negative values pull stuff in (intake)
         */
        set(value) {
            intakeMotor.power = Range.clip(value, -1.0, 1.0) * IntakePower.coefficient
        }
        get() {
            return intakeMotor.power / IntakePower.coefficient
        }

    init {
        intakeMotor = hardwareMap.get(DcMotorEx::class.java, Motors.INTAKE.name)
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.power = 0.0
    }
}