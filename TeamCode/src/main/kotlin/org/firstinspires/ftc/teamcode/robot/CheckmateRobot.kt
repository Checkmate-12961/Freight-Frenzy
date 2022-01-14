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
package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot
import org.firstinspires.ftc.teamcode.robot.util.LynxModuleUtil
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.robot.subsystems.*
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.Barcode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.Drivetrain

/**
 * Checkmate robot class to access subsystems.
 *
 * @constructor
 * Enables Lynx caching. Registers default subsystems.
 *
 * @param hardwareMap Passed in from [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
 */
class CheckmateRobot(hardwareMap: HardwareMap) : AbstractRobot() {
    override val tag = "CheckmateRobot"

    /**
     * Access the [Drivetrain] subsystem from the registry.
     */
    val drivetrain: Drivetrain
        get() = subsystems["Drivetrain"] as Drivetrain
    /**
     * Access the [Carousel] subsystem from the registry.
     */
    val carousel: Carousel
        get() = subsystems["Carousel"] as Carousel
    /**
     * Access the [Lift] subsystem from the registry.
     */
    val lift: Lift
        get() = subsystems["Lift"] as Lift
    /**
     * Access the [Intake] subsystem from the registry.
     */
    val intake: Intake
        get() = subsystems["Intake"] as Intake
    /**
     * Access the [Bucket] subsystem from the registry.
     */
    val bucket: Bucket
        get() = subsystems["Bucket"] as Bucket
    /**
     * Access the [Barcode] subsystem from the registry.
     */
    val barcode: Barcode
        get() = subsystems["Barcode"] as Barcode

    init {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // Set up the drivetrain
        subsystems.register(Drivetrain(hardwareMap))

        // Set up the carousel motor
        subsystems.register(Carousel(hardwareMap))

        // Set up the intake
        subsystems.register(Intake(hardwareMap))

        // Set up the bucket
        subsystems.register(Bucket(hardwareMap))

        // Set up the lift (it needs access to the bucket and the intake)
        subsystems.register(Lift(hardwareMap, bucket, intake))

        // Set up the barcode stuff
        subsystems.register(Barcode(hardwareMap))
    }
}