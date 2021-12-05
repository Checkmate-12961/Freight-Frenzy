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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

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
            OpModeMeta.Builder metaBuilder = new OpModeMeta.Builder()
                    .setGroup("tuning")
                    .setFlavor(OpModeMeta.Flavor.AUTONOMOUS);

            // i am sad
            manager.register(
                    metaBuilder.setName(AutomaticFeedforwardTuner.class.getSimpleName()).build(),
                    new AutomaticFeedforwardTuner());
            manager.register(
                    metaBuilder.setName(BackAndForth.class.getSimpleName()).build(),
                    new BackAndForth());
            manager.register(
                    metaBuilder.setName(FollowerPIDTuner.class.getSimpleName()).build(),
                    new FollowerPIDTuner());
            manager.register(
                    metaBuilder.setName(LocalizationTest.class.getSimpleName()).build(),
                    new LocalizationTest());
            manager.register(
                    metaBuilder.setName(MaxVelocityTuner.class.getSimpleName()).build(),
                    new MaxVelocityTuner());
            manager.register(
                    metaBuilder.setName(SplineTest.class.getSimpleName()).build(),
                    new SplineTest());
            manager.register(
                    metaBuilder.setName(StrafeTest.class.getSimpleName()).build(),
                    new StrafeTest());
            manager.register(
                    metaBuilder.setName(StraightTest.class.getSimpleName()).build(),
                    new StraightTest());
            manager.register(
                    metaBuilder.setName(TrackWidthTuner.class.getSimpleName()).build(),
                    new TrackWidthTuner());
            manager.register(
                    metaBuilder.setName(TurnTest.class.getSimpleName()).build(),
                    new TurnTest());
            metaBuilder.setFlavor(OpModeMeta.Flavor.TELEOP);
            manager.register(
                    metaBuilder.setName(ManualFeedforwardTuner.class.getSimpleName()).build(),
                    new ManualFeedforwardTuner());
            manager.register(
                    metaBuilder.setName(TrackingWheelLateralDistanceTuner.class.getSimpleName())
                            .build(),
                    new TrackingWheelLateralDistanceTuner());
        }
    }
}
