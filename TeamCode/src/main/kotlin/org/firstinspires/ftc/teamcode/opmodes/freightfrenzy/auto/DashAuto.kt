package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

@Autonomous
class DashAuto : BaseOpMode() {
    enum class Action(val action: (TrajectorySequenceBuilder, Double) -> Unit) {
        FORWARD({ builder, magnitude ->
            builder.forward(magnitude)
        }),
        BACK({ builder, magnitude ->
            builder.back(magnitude)
        }),
        TURN({ builder, magnitude ->
            builder.turn(magnitude)
        });

        fun invoke(builder: TrajectorySequenceBuilder, magnitude: Double) {
            action.invoke(builder, magnitude)
        }
        fun fromId(id: String): Action {
            return Action.valueOf(id.uppercase())
        }
    }

    data class ActionBuilder(@JvmField var id: String, @JvmField var magnitude: Double)

    companion object {
        @JvmField var a0_action = ActionBuilder("", 0.0)
        @JvmField var a1_action = ActionBuilder("", 0.0)
        @JvmField var a2_action = ActionBuilder("", 0.0)
        @JvmField var a3_action = ActionBuilder("", 0.0)
        @JvmField var a4_action = ActionBuilder("", 0.0)

        @JvmField var b0_carouselPower = 1.0
        @JvmField var b1_carouselTime = 2000

        @JvmField var c0_action = ActionBuilder("", 0.0)
        @JvmField var c1_action = ActionBuilder("", 0.0)
        @JvmField var c2_action = ActionBuilder("", 0.0)
        @JvmField var c3_action = ActionBuilder("", 0.0)
        @JvmField var c4_action = ActionBuilder("", 0.0)
    }
}