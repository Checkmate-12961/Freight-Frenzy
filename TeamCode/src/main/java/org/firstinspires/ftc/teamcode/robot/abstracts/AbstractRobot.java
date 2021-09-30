package org.firstinspires.ftc.teamcode.robot.abstracts;

import java.util.ArrayList;

/**
 * Class that all robots need to extend
 */
public abstract class AbstractRobot {
    private final ArrayList<AbstractSubsystem> subsystems;

    public AbstractRobot(){
        subsystems = new ArrayList<>();
    }

    /**
     * Updates all subsystems
     */
    public final void update() {
        for (AbstractSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    /**
     * Runs cleanup() on all subsystems
     */
    public final void cleanup() {
        for (AbstractSubsystem subsystem : subsystems) {
            subsystem.cleanup();
        }
    }

    /**
     * Adds a new subsystem
     * @param subsystem Subsystem to add. Must implement the Subsystem class
     */
    protected final void addSubsystem(AbstractSubsystem subsystem){
        subsystems.add(subsystem);
    }
}
