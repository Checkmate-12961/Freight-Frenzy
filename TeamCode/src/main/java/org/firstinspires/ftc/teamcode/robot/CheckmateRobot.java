package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

/**
 * The 2021-2022 robot class
 */
public class CheckmateRobot extends AbstractRobot {
    public Drivetrain drivetrain;
    public Lift lift;

    /**
     * Set up the robot and initialize the subsystems you want to use
     * @param hardwareMap Passed in from the op mode
     */
    public CheckmateRobot(HardwareMap hardwareMap) {
        super();

        // Set up the drivetrain
        drivetrain = new Drivetrain(hardwareMap);
        addSubsystem(drivetrain);

        // Set up the lift
        lift = new Lift(hardwareMap);
        addSubsystem(lift);
    }
}
