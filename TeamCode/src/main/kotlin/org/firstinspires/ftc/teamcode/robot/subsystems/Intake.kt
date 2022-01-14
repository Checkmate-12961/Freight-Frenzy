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
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

/**
 * Subsystem to manage the intake mechanism.
 *
 * @constructor
 * Initialize the motor and set its direction.
 *
 * @param hardwareMap Passed in from [org.firstinspires.ftc.teamcode.robot.CheckmateRobot].
 */
@Config
class Intake(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "Intake"
    override val subsystems = SubsystemMap{ tag }

    private val intakeMotor = Motors.INTAKE.get(hardwareMap)

    companion object{
        @JvmField var coefficient = .7
    }

    /**
     * Power of the intake motor. Positive values push stuff out, negative values pull stuff in.
     */
    var power: Double
        set(value) {
            intakeMotor.power = Range.clip(value, -1.0, 1.0) * coefficient
        }
        get() {
            return intakeMotor.power / coefficient
        }

    init {
        // Set up the intake motor
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.power = 0.0
    }
}