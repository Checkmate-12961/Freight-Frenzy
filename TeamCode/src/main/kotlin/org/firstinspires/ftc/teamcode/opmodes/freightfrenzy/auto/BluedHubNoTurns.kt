package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import kotlin.math.PI

@Autonomous
class BluedHubNoTurns : BaseOpMode() {
    private lateinit var seq0: TrajectorySequence
    private lateinit var seq1: TrajectorySequence

    override fun preSetup() {
        Drivetrain.useAlternateLocalizer = true
    }

    private val startPose = Pose2d(-54.0, 61.5, PI /2)

    override fun setup() {
        seq0 = robot.drivetrain.trajectorySequenceBuilder(startPose)
            .back(15.0)
            .strafeRight(42.5)
            .back(5.0)
            .build()
        seq1 = robot.drivetrain.trajectorySequenceBuilder(seq0.end())
            .forward(5.0)
            .strafeLeft(42.5)
            .back(11.0)
            .strafeLeft(8.0)
            .build()

        robot.drivetrain.poseEstimate = startPose
    }

    override fun runLoop() {
        when (segment) {
            Segment.ZERO -> {
                if (!seqSet) {
                    robot.drivetrain.poseEstimate = startPose
                    robot.drivetrain.followTrajectorySequenceAsync(seq0)
                    seqSet = true
                } else if (!robot.drivetrain.isBusy) {
                    segment = segment.next()
                    seqSet = false
                }
            }
            Segment.LIFT -> {
                if (timerStep == 0) {
                    robot.lift.target = Lift.Points.HIGH
                    timerNext()
                } else if (timerStep == 1) {
                    if (timeDelta > 2000) {
                        robot.bucket.position = Bucket.Positions.DUMP
                        timerNext()
                    }
                } else if (timerStep == 2) {
                    if (timeDelta > 3500) {
                        robot.lift.target = Lift.Points.MIN
                        timerNext()
                    }
                } else {
                    timerReset()
                    segment = segment.next()
                }
            }
            Segment.ONE -> {
                if (!seqSet) {
                    robot.drivetrain.followTrajectorySequenceAsync(seq1)
                    seqSet = true
                } else if (!robot.drivetrain.isBusy) {
                    segment = segment.next()
                    seqSet = false
                }
            }
            else -> requestOpModeStop()
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

    private var seqSet = false
    private var segment = Segment.ZERO

    enum class Segment {
        ZERO, LIFT, ONE, DONE;

        fun next(): Segment {
            return when(this) {
                ZERO -> LIFT
                LIFT -> ONE
                else -> DONE
            }
        }
    }
}