package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.teamcode.opmodes.tuning.AutomaticFeedforwardTuner;
import org.firstinspires.ftc.teamcode.opmodes.tuning.BackAndForth;
import org.firstinspires.ftc.teamcode.opmodes.tuning.FollowerPIDTuner;
import org.firstinspires.ftc.teamcode.opmodes.tuning.LocalizationTest;
import org.firstinspires.ftc.teamcode.opmodes.tuning.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.opmodes.tuning.MaxVelocityTuner;
import org.firstinspires.ftc.teamcode.opmodes.tuning.SplineTest;
import org.firstinspires.ftc.teamcode.opmodes.tuning.StrafeTest;
import org.firstinspires.ftc.teamcode.opmodes.tuning.StraightTest;
import org.firstinspires.ftc.teamcode.opmodes.tuning.TrackWidthTuner;
import org.firstinspires.ftc.teamcode.opmodes.tuning.TrackingWheelLateralDistanceTuner;
import org.firstinspires.ftc.teamcode.opmodes.tuning.TurnTest;

// stupid switch to enable all of the tuning op modes
public class TuningMode {
    public static final boolean isTuning = false; // make this true for tuning mode and false for not

    @OpModeRegistrar
    public static void registerTuningOpModes(OpModeManager manager) {
        if (isTuning) {
            // i am sad
            manager.register(AutomaticFeedforwardTuner.class.getSimpleName(), new AutomaticFeedforwardTuner());
            manager.register(BackAndForth.class.getSimpleName(), new BackAndForth());
            manager.register(FollowerPIDTuner.class.getSimpleName(), new FollowerPIDTuner());
            manager.register(LocalizationTest.class.getSimpleName(), new LocalizationTest());
            manager.register(ManualFeedforwardTuner.class.getSimpleName(), new ManualFeedforwardTuner());
            manager.register(MaxVelocityTuner.class.getSimpleName(), new MaxVelocityTuner());
            manager.register(SplineTest.class.getSimpleName(), new SplineTest());
            manager.register(StrafeTest.class.getSimpleName(), new StrafeTest());
            manager.register(StraightTest.class.getSimpleName(), new StraightTest());
            manager.register(TrackingWheelLateralDistanceTuner.class.getSimpleName(), new TrackingWheelLateralDistanceTuner());
            manager.register(TrackWidthTuner.class.getSimpleName(), new TrackWidthTuner());
            manager.register(TurnTest.class.getSimpleName(), new TurnTest());
        }
    }
}
