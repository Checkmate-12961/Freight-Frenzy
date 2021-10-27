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

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.RealsenseLoader;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.RealsenseLoader;

/**
 * The 2021-2022 robot class
 */
public class CheckmateRobot extends AbstractRobot {
    public final Drivetrain drivetrain;
    public Drivetrain drivetrain;
    public Carousel carousel;
    public Lift lift;

    /**
     * Set up the robot and initialize the subsystems you want to use
     * @param hardwareMap Passed in from the op mode
     */
    public CheckmateRobot(HardwareMap hardwareMap) {
        super();

        // Set up the Realsense camera
        addSubsystem(new RealsenseLoader(hardwareMap));

        // Set up the drivetrain
        drivetrain = new Drivetrain(hardwareMap);
        addSubsystem(drivetrain);

        // Set up the carousel motor
        carousel = new Carousel(hardwareMap);
        addSubsystem(carousel);

        // Set up the lift
        lift = new Lift(hardwareMap);
        addSubsystem(lift);
    }
}
