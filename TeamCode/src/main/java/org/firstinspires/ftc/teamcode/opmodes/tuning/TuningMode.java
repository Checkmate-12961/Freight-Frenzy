package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/**
 * Awful class that enables or disables all tuning OpModes at once
 */
@SuppressWarnings("unused")
public class TuningMode {
    // true for tuning mode on and false for tuning mode off
    public static final boolean isTuning = false;

    /**
     * Registers tuning OpModes
     * @param manager OpModeManager passed in by the OpMode register
     */
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
