package org.firstinspires.ftc.teamcode.robot.abstracts;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public abstract class AbstractRobot {
    private final ArrayList<AbstractSubsystem> subsystems;

    public AbstractRobot(){
        subsystems = new ArrayList<>();
    }

    /**
     * Updates all subsystems
     */
    public void update() {
        for (AbstractSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    /**
     * Runs cleanup() on all subsystems
     */
    public void cleanup() {
        for (AbstractSubsystem subsystem : subsystems) {
            subsystem.cleanup();
        }
    }

    /**
     * Adds a new subsystem
     * @param subsystem Subsystem to add. Must implement the Subsystem class
     */
    protected void addSubsystem(AbstractSubsystem subsystem){
        subsystems.add(subsystem);
    }
}
