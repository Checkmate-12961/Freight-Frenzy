package org.firstinspires.ftc.teamcode.robot.abstracts;

public interface AbstractSubsystem {
    /**
     * Runs on each iteration of the main Op Mode loop
     */
    void update();

    /**
     * Runs when an Op Mode ends
     */
    default void cleanup() {}
}
