package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.RealsenseLoader;

/**
 * The 2021-2022 robot class
 */
public class CheckmateRobot extends AbstractRobot {
    public final Drivetrain drivetrain;

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
    }
}
