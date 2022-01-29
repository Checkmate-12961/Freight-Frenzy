package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.util.OpModeUtil
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder

@Disabled
@Autonomous
class DashAuto : BaseOpMode() {
    private lateinit var traj0: TrajectorySequence
    private lateinit var traj1: TrajectorySequence
    private lateinit var traj2: TrajectorySequence

    private val traj0ActionBuilders: Array<ActionBuilder>
        get() = arrayOf(a0_action, a1_action, a2_action, a3_action, a4_action)
    private val traj1ActionBuilders: Array<ActionBuilder>
        get() = arrayOf(c0_action, c1_action, c2_action, c3_action, c4_action)
    private val traj2ActionBuilders: Array<ActionBuilder>
        get() = arrayOf(e0_action, e1_action, e2_action, e3_action, e4_action)

    override fun setup() {
        val trajBuilder0 = robot.drivetrain.trajectorySequenceBuilder(z_startPose.pose2d)
        for (actionBuilder in traj0ActionBuilders) {
            if (actionBuilder.id != "") {
                Action.fromId(actionBuilder.id).invoke(trajBuilder0, actionBuilder.magnitude)
            }
        }
        traj0 = trajBuilder0.build()


        val trajBuilder1 = robot.drivetrain.trajectorySequenceBuilder(traj0.end())
        for (actionBuilder in traj1ActionBuilders) {
            if (actionBuilder.id != "") {
                Action.fromId(actionBuilder.id).invoke(trajBuilder1, actionBuilder.magnitude)
            }
        }
        traj1 = trajBuilder1.build()


        val trajBuilder2 = robot.drivetrain.trajectorySequenceBuilder(traj1.end())
        for (actionBuilder in traj2ActionBuilders) {
            if (actionBuilder.id != "") {
                Action.fromId(actionBuilder.id).invoke(trajBuilder2, actionBuilder.magnitude)
            }
        }
        traj0 = trajBuilder2.build()
    }

    override fun runLoop() {
        when(segment) {
            Segments.ZERO -> {
                if (!trajSet && !robot.drivetrain.isBusy) {
                    trajSet = true
                    robot.drivetrain.followTrajectorySequenceAsync(traj0)
                } else if (trajSet && !robot.drivetrain.isBusy) {
                    trajSet = false
                    segmentNext()
                }
            }
            Segments.CAROUSEL -> {
                if (timerStep == 0) {
                    robot.carousel.power = b0_carouselPower
                    timerNext()
                } else if (timerStep == 1) {
                    if (timeDelta > b1_carouselTime) {
                        robot.carousel.power = 0.0
                        timerNext()
                    }
                } else {
                    timerReset()
                    segmentNext()
                }
            }
            Segments.ONE -> {
                if (!trajSet && !robot.drivetrain.isBusy) {
                    trajSet = true
                    robot.drivetrain.followTrajectorySequenceAsync(traj1)
                } else if (trajSet && !robot.drivetrain.isBusy) {
                    trajSet = false
                    segmentNext()
                }
            }
            Segments.LIFT -> {
                if (timerStep == 0) {
                    robot.lift.target = robot.barcode.position.toLiftPoint()
                    timerNext()
                } else if (timerStep == 1) {
                    if (timeDelta > d0_liftTime) {
                        robot.bucket.position = Bucket.Positions.DUMP
                        timerNext()
                    }
                } else if (timerStep == 2) {
                    if (timeDelta > d1_bucketTime) {
                        robot.lift.target = Lift.Points.MIN
                        timerNext()
                    }
                } else {
                    timerReset()
                    segmentNext()
                }
            }
            Segments.TWO -> {
                if (!trajSet && !robot.drivetrain.isBusy) {
                    trajSet = true
                    robot.drivetrain.followTrajectorySequenceAsync(traj2)
                } else if (trajSet && !robot.drivetrain.isBusy) {
                    trajSet = false
                    segmentNext()
                }
            }
            Segments.DONE -> requestOpModeStop()
        }
    }

    private fun timerNext() {
        lastTime = currentTime
        timerStep++
    }

    private fun timerReset() {
        lastTime = 0
        timerStep = 0
    }

    private var timerStep = 0
    private var lastTime = 0L
    private val currentTime: Long
        get() = System.currentTimeMillis()
    private val timeDelta: Long
        get() = currentTime - lastTime

    private fun segmentNext() {
        segment = segment.next()
    }

    private var segment = Segments.ZERO
    private var trajSet = false

    enum class Segments {
        ZERO, CAROUSEL, ONE, LIFT, TWO, DONE;

        fun next(): Segments {
            return when(this) {
                ZERO -> CAROUSEL
                CAROUSEL -> ONE
                ONE -> LIFT
                LIFT -> TWO
                else -> DONE
            }
        }
    }

    enum class Action(val action: (TrajectorySequenceBuilder, Double) -> Unit) {
        FORWARD({ builder, magnitude ->
            builder.forward(magnitude)
        }),
        BACK({ builder, magnitude ->
            builder.back(magnitude)
        }),
        LEFT({ builder, magnitude ->
            builder.strafeLeft(magnitude)
        }),
        RIGHT({ builder, magnitude ->
            builder.strafeRight(magnitude)
        }),
        TURN({ builder, magnitude ->
            builder.turn(magnitude)
        });

        fun invoke(builder: TrajectorySequenceBuilder, magnitude: Double) {
            action.invoke(builder, magnitude)
        }

        companion object {
            fun fromId(id: String): Action {
                return Action.valueOf(id.uppercase())
            }
        }
    }

    data class ActionBuilder(@JvmField var id: String, @JvmField var magnitude: Double)

    companion object {
        @JvmField var z_startPose = OpModeUtil.SuperPose2d(0.0, 0.0, 0.0)

        @JvmField var a0_action = ActionBuilder("", 0.0)
        @JvmField var a1_action = ActionBuilder("", 0.0)
        @JvmField var a2_action = ActionBuilder("", 0.0)
        @JvmField var a3_action = ActionBuilder("", 0.0)
        @JvmField var a4_action = ActionBuilder("", 0.0)

        @JvmField var b0_carouselPower = 1.0
        @JvmField var b1_carouselTime = 2000L

        @JvmField var c0_action = ActionBuilder("", 0.0)
        @JvmField var c1_action = ActionBuilder("", 0.0)
        @JvmField var c2_action = ActionBuilder("", 0.0)
        @JvmField var c3_action = ActionBuilder("", 0.0)
        @JvmField var c4_action = ActionBuilder("", 0.0)

        @JvmField var d0_liftTime = 1500L
        @JvmField var d1_bucketTime = 2500L

        @JvmField var e0_action = ActionBuilder("", 0.0)
        @JvmField var e1_action = ActionBuilder("", 0.0)
        @JvmField var e2_action = ActionBuilder("", 0.0)
        @JvmField var e3_action = ActionBuilder("", 0.0)
        @JvmField var e4_action = ActionBuilder("", 0.0)
    }
}