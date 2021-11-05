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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;

/**
 * Basic OpMode class that all OpModes should extend
 */
public abstract class BaseOpMode extends OpMode {
    protected CheckmateRobot robot;

    protected  SuperController gp1;
    protected  SuperController gp2;

    protected enum OpModeType {
        TeleOp,
        Autonomous
    }

    protected OpModeType opModeType = null;

    /**
     * Internal method, do not call or overwrite
     */
    @Override
    final public void init() {
        if (this.getClass().getAnnotation(TeleOp.class) != null) {
            opModeType = OpModeType.TeleOp;
        } else if (this.getClass().getAnnotation(Autonomous.class) != null) {
            opModeType = OpModeType.Autonomous;
        }

        pre_setup();
        robot = new CheckmateRobot(hardwareMap);
        gp1 = new SuperController(gamepad1);
        gp2 = new SuperController(gamepad2);
        setup();
    }

    /**
     * Runs before the hardware initializes
     */
    public void pre_setup() { }

    /**
     * Runs when the OpMode initializes
     */
    public abstract void setup();

    /**
     * Internal method, do not call or overwrite
     */
    @Override
    final public void loop() {
        if (opModeType == OpModeType.TeleOp) {
            gp1.update();
            gp2.update();
        }

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
