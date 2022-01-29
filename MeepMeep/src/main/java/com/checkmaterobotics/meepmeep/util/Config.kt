package com.checkmaterobotics.meepmeep.util

import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

object Config {
    fun get(): MeepMeep {
        System.setProperty("java.awt.headless", "false")

        // With a field size of 800 pixels
        return MeepMeep(600)
            // Set field image
            .setBackground(Background.FIELD_FREIGHT_FRENZY)
            // Set theme
            .setTheme(ColorSchemeRedDark())
            // Background opacity from 0-1
            .setBackgroundAlpha(1f)
            .setBotDimensions(
                13.25,
                17.5
            ) // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(35.0, 30.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.0)
    }

    fun start(callback: (CheckmateRobot) -> TrajectorySequence) {
        get().followTrajectorySequence {
            callback.invoke(CheckmateRobot(it))
        }.start()
    }

    class Pose2d(val x: Double, val y: Double, val heading: Double) {

    }
}