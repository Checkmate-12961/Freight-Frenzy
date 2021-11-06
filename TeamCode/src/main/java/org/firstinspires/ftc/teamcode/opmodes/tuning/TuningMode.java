/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
