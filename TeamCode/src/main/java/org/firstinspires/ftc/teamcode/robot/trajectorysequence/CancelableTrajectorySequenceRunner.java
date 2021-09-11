package org.firstinspires.ftc.teamcode.robot.trajectorysequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;

@Config
public class CancelableTrajectorySequenceRunner extends TrajectorySequenceRunner{
    public CancelableTrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        super(follower, headingPIDCoefficients);
    }

    public void cancelSequence() {
        super.currentTrajectorySequence = null;
    }
}
