package org.firstinspires.ftc.teamcode.robot.abstracts;

public interface AbstractSubsystem {
    default void update() {}

    default void cleanup() {}
}
