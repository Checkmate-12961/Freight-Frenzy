package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Disabled
@Autonomous(preselectTeleOp = "TeleOp")
@Config
class JustGo : BaseOpMode() {
    lateinit var traj: TrajectorySequence

    override fun setup() {
        // Make an array of the poses so we can iterate over them
        val poses = arrayOf(pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9)

        // Initialize a trajectory builder
        val trajBuilder = robot.drivetrain.trajectorySequenceBuilder(pose0.toPose2d())

        // Add poses that have been set to the trajectory builder
        for (pose in poses) {
            if (pose.x != 0.0 || pose.y != 0.0 || pose.h != 0.0) {
                trajBuilder.splineTo(pose.toVector2d(), pose.hR)
            } else break
        }

        // Build the trajectory
        traj = trajBuilder.build()
    }

    override fun preRunLoop() {
        // Follow the trajectory
        robot.drivetrain.followTrajectorySequenceAsync(traj)
    }

    companion object {
        @JvmField var pose0: Pose = Pose()
        @JvmField var pose1: Pose = Pose()
        @JvmField var pose2: Pose = Pose()
        @JvmField var pose3: Pose = Pose()
        @JvmField var pose4: Pose = Pose()
        @JvmField var pose5: Pose = Pose()
        @JvmField var pose6: Pose = Pose()
        @JvmField var pose7: Pose = Pose()
        @JvmField var pose8: Pose = Pose()
        @JvmField var pose9: Pose = Pose()
    }

    class Pose(@JvmField var x: Double = 0.0,
               @JvmField var y: Double = 0.0,
               @JvmField var h: Double = 0.0) {

        /**
         * Convert to a Pose2d
         *
         * @return x, y, and h in radians as a Pose2d
         */
        fun toPose2d(): Pose2d {
            return Pose2d(x, y, hR)
        }

        /**
         * Convert to a Vector2d
         *
         * @return x and y in a Vector2d
         */
        fun toVector2d(): Vector2d {
            return Vector2d(x, y)
        }

        /**
         * Heading in radians
         */
        val hR: Double
            get() = h * kotlin.math.PI / 180.0
    }
}
