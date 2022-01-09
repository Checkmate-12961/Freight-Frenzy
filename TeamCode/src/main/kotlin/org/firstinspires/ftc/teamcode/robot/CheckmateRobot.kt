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

/**
 * The 2021-2022 robot class
 */
class CheckmateRobot(hardwareMap: HardwareMap) : AbstractRobot() {
    val drivetrain: Drivetrain
    val carousel: Carousel
    //val lift: Lift
    val intake: Intake
    val bucket: Bucket
    val barcode: Barcode
    //val capper: Capper
    val shitLift: LiftPower

    /**
     * Set up the robot and initialize the subsystems you want to use
     */
    init {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // Set up the drivetrain
        drivetrain = Drivetrain(hardwareMap)
        addSubsystem(drivetrain)

        //addSubsystem(new RealsenseLoader(hardwareMap));

        // Set up the carousel motor
        carousel = Carousel(hardwareMap)
        addSubsystem(carousel)

        // Set up the intake
        intake = Intake(hardwareMap)
        addSubsystem(intake)

        // Set up the bucket
        bucket = Bucket(hardwareMap)
        addSubsystem(bucket)

        // Set up the lift (it needs access to the bucket)
        shitLift = LiftPower(hardwareMap, bucket, intake)
        addSubsystem(shitLift)

        // Set up the barcode stuff
        barcode = Barcode(hardwareMap)
        addSubsystem(barcode)

        // Set up the capper stuff
        //capper = Capper(hardwareMap)
        //addSubsystem(capper)
    }
}