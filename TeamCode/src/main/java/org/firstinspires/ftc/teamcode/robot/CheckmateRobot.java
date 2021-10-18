package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.RealsenseLoader;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

/**
 * The 2021-2022 robot class
 */
public class CheckmateRobot extends AbstractRobot {
    public final Drivetrain drivetrain;
    public Drivetrain drivetrain;
    public Carousel carousel;
    public Lift lift;

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

        // Set up the carousel motor
        carousel = new Carousel(hardwareMap);
        addSubsystem(carousel);

        // Set up the lift
        lift = new Lift(hardwareMap);
        addSubsystem(lift);
    }
}
