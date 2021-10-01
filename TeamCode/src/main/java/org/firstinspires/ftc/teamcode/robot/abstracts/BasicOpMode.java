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
