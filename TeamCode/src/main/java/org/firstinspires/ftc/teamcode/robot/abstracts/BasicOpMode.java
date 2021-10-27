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

package org.firstinspires.ftc.teamcode.robot.abstracts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;

/**
 * Basic OpMode class that all OpModes should extend
 */
public abstract class BasicOpMode extends OpMode {
    protected CheckmateRobot robot;

    /**
     * Internal method, do not call or overwrite
     */
    @Override
    final public void init() {
        robot = new CheckmateRobot(hardwareMap);
        setup();
    }

    /**
     * Runs when the OpMode initializes
     */
    public abstract void setup();

    /**
     * Internal method, do not call or overwrite
     */
    @Override
    final public void loop() {
        robot.update();
        run_loop();
    }

    /**
     * Main OpMode loop, automatically updates the robot
     */
    public abstract void run_loop();

    /**
     * Internal method, do not call or overwrite
     */
    @Override
    final public void stop() {
        robot.cleanup();
        cleanup();
    }

    /**
     * Optional, runs when the OpMode is stopped
     */
    public void cleanup() {

    }
}
