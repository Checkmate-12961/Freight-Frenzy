package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.util.OpModeUtil
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer
import kotlin.math.PI

/**
 * Helps tune T265.
 */
@Config
@TeleOp(group = "tuning")
class TuneT265 : BaseOpMode() {
    override fun preSetup() {
        Drivetrain.useAlternateLocalizer = true
    }

    private var spin = false

    override fun setup() {
        gp1.a.onActivate = {spin = true}
        gp2.a.onActivate = gp1.a.onActivate

        gp1.b.onActivate = {spin = false}
        gp2.b.onActivate = gp1.b.onActivate
    }

    override fun runLoop() {
        if (!robot.drivetrain.isBusy && spin) {
            robot.drivetrain.turnAsync(PI)
        } else if (robot.drivetrain.isBusy && !spin) {
            robot.drivetrain.setWeightedDrivePower(Pose2d())
        }
    }
}