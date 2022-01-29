package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.util.OpModeUtil
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer
import kotlin.math.PI

/**
 * Helps tune T265.
 */
@Config
@TeleOp(group = "tuning")
class TuneT265 : BaseOpMode() {
    private var spin = false
    private var lastOffset = cameraRobotOffset
    private val offsetChanged: Boolean
        get() {
            return (lastOffset.x - cameraRobotOffset.x) > offsetChangedThreshold ||
                    (lastOffset.y - cameraRobotOffset.y) > offsetChangedThreshold ||
                    (lastOffset.h - cameraRobotOffset.h) > offsetChangedThreshold
        }

    override fun setup() {
        gp1.a.onActivate = {spin = true}
        gp2.a.onActivate = gp1.a.onActivate

        gp1.b.onActivate = {spin = false}
        gp2.b.onActivate = gp1.b.onActivate
    }

    override fun runLoop() {
        if (offsetChanged) {
            lastOffset = cameraRobotOffset
            (robot.drivetrain.localizer as T265Localizer).setOdometryInfo(cameraRobotOffset.pose2d)
        }
        if (!robot.drivetrain.isBusy && spin) {
            robot.drivetrain.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.5))
        } else if (robot.drivetrain.isBusy && !spin) {
            robot.drivetrain.setWeightedDrivePower(Pose2d())
        }
    }

    companion object {
        @JvmField var cameraRobotOffset = OpModeUtil.SuperPose2d(-5.0, -2.75, 90.0)
        @JvmField var offsetChangedThreshold = 0.2
    }
}