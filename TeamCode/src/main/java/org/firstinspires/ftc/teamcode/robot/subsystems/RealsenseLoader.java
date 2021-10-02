package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.realsenseloader.RealsenseManager;

public class RealsenseLoader implements AbstractSubsystem {
    /**
     * Loads up the realsense camera
     */
    public RealsenseLoader(HardwareMap hardwareMap) {
        RealsenseManager.init(hardwareMap);
    }

    /**
     * Doesn't do anything.
     */
    @Override
    public void update() { }

    /**
     * Shut down the camera
     */
    @Override
    public void cleanup() {
        RealsenseManager.cleanup();
    }
}
