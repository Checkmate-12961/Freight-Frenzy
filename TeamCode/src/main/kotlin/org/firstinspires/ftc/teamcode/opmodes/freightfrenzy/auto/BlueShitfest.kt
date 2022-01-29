package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import kotlin.math.PI

@Autonomous(name = "BLUE")
class BlueShitfest: BaseOpMode() {
    enum class Segment {
        ZERO, CARUOUSEL, ONE, LIFT, TWO, BUSY
    }

    var segment = Segment.ZERO

    lateinit var traj0: TrajectorySequence
    lateinit var traj1: TrajectorySequence
    lateinit var traj2: TrajectorySequence

    override fun setup() {
        traj0 = robot.drivetrain.trajectorySequenceBuilder(Pose2d())
            .back(-10.0)
            .turn(-PI/2)
            .back(30.0)
            .build()
    }

    override fun preRunLoop() {
        robot.drivetrain.poseEstimate = traj0.start()
        robot.drivetrain.followTrajectorySequenceAsync(traj0)
    }

    override fun runLoop() {
        when(segment) {
            Segment.ZERO -> {
                if (!robot.drivetrain.isBusy) {
                    segment = Segment.CARUOUSEL
                }
            }
            Segment.CARUOUSEL -> {
                robot.carousel.power = 1.0
                sleep(2000)
                robot.carousel.power = 0.0
                segment = Segment.ONE
            }
        }
    }
}