package org.firstinspires.ftc.teamcode.robot.abstracts;

/**
 * Interface that all subsystems need to implement
 */
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
