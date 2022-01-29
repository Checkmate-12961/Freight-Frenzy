package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import kotlin.math.PI
import kotlin.math.abs

@Autonomous
class BlueSide : BaseOpMode() {
    override fun setupLoop() {
        telemetry.addData("confidence", (robot.drivetrain.localizer as T265Localizer).poseConfidence)
    }
    private val startPose = Pose2d(-34.25, 62.0, PI/2)

    enum class Segment { ZERO, CAROUSEL, ONE, LIFT, TWO, BUSY }

    private var segment = Segment.ZERO
        set(value) {
            field = value
            Log.d("BlueSide", "Segment: $value")
        }

    private var lastTime = 0.0
    private val currentTime: Double
        get() = NanoClock.system().seconds()
    private val timeDelta: Double
        get() = currentTime - lastTime

    private lateinit var trajectory0: TrajectorySequence
    private lateinit var trajectory1: TrajectorySequence
    private lateinit var trajectory2: TrajectorySequence

    override fun setup() {
        trajectory0 = robot.drivetrain.trajectorySequenceBuilder(startPose)
            .back(4.0)
            .turn(-PI/2)
            .lineToSplineHeading(Pose2d(-52.0, 56.0, 0.0))
            .addDisplacementMarker {
                segment = Segment.CAROUSEL
            }
            .build()
        trajectory1 = robot.drivetrain.trajectorySequenceBuilder(trajectory0.end())
            .splineTo(Vector2d(-58.0, 36.0), -PI/2)
            .splineTo(Vector2d(-29.0, 25.0), 0.0)
            .addDisplacementMarker{
                segment = Segment.LIFT
            }
            .build()
        trajectory2 = robot.drivetrain.trajectorySequenceBuilder(trajectory1.end())
            .lineTo(Vector2d(-40.0, 25.0))
            .splineTo(Vector2d(-58.0, 36.0), PI/2)
            .build()
        var pose = robot.drivetrain.poseEstimate
        while (abs(abs(pose.x) - abs(startPose.x) + abs(pose.y) - abs(startPose.y) + abs(pose.heading) - abs(startPose.heading)) > 2.0) {
            Log.d("BlueSide", "${pose.x}, ${pose.y}, ${pose.heading}")
            robot.drivetrain.poseEstimate = startPose
            pose = robot.drivetrain.poseEstimate
        }
    }

    override fun preRunLoop() {
        robot.drivetrain.poseEstimate = startPose
    }

    override fun runLoop() {
        telemetry.addData("Segment", segment)
        when (segment) {
            Segment.ZERO -> {
                segment = Segment.BUSY
                robot.drivetrain.followTrajectorySequenceAsync(trajectory0)
            }
            Segment.CAROUSEL -> {
                if (lastTime == 0.0) {
                    lastTime = currentTime
                    robot.carousel.power = 1.0
                } else if (timeDelta > 1.5) {
                    lastTime = 0.0
                    robot.carousel.power = 0.0
                    segment = Segment.ONE
                }
            }
            Segment.ONE -> {
                segment = Segment.BUSY
                robot.drivetrain.followTrajectorySequenceAsync(trajectory1)
            }
            Segment.LIFT -> {
                if (lastTime == 0.0) {
                    lastTime = currentTime
                    robot.lift.target = robot.barcode.position.toLiftPoint()
                } else if (timeDelta > 3.5) {
                    lastTime = 0.0
                    segment = Segment.TWO
                    robot.lift.target = Lift.Points.MIN
                } else if (timeDelta > 2.5) {
                    robot.bucket.position = Bucket.Positions.DUMP
                }
            }
            Segment.TWO -> {
                segment = Segment.BUSY
                robot.drivetrain.followTrajectorySequenceAsync(trajectory2)
            }
            else -> {}
        }
    }
}