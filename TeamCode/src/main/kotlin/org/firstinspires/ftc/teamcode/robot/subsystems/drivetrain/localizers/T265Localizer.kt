package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.geometry.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import java.util.function.Consumer
import kotlin.math.cos
import kotlin.math.sin

/**
 * Transforms a Pose2d by another Pose2d.
 * [A simulator for this behavior can be found here.](https://www.desmos.com/calculator/0x7xoeiudh)
 *
 * @param b The pose to transform the first pose by.
 * @return The result of the transformation.
 */
fun Pose2d.transformBy(b: Pose2d): Pose2d {
    val cosT = cos(heading)
    val sinT = sin(heading)
    return Pose2d(
        x + b.x * cosT - b.y * sinT,
        y + b.x * sinT + b.y * cosT,
        heading + b.heading
    )
}

/**
 * Calculates the transformation between two Pose2d objects.
 * [A simulator for this behavior can be found here.](https://www.desmos.com/calculator/fttw74j9pp)
 *
 * @param r The result pose.
 * @return The transformation needed to get from the origin pose to the result pose.
 */
fun Pose2d.calculateTransformation(r: Pose2d): Pose2d {
    val cosT = cos(heading)
    val sinT = sin(heading)
    return Pose2d(
        cosT * (r.x - x) + sinT * (r.y - y),
        sinT * (x - r.x) + cosT * (r.y - y),
        r.heading - heading
    )
}

class T265Localizer(
    cameraToRobot: Pose2d,
    odometryCovariance: Double,
    hardwareMap: HardwareMap,
): Localizer, Consumer<T265Camera.CameraUpdate>, AbstractSubsystem {
    override val tag = "T265Localizer"
    override val subsystems = SubsystemMap{ tag }

    // SLAMERA STUFF //
    var odometryVelocityCallback: (() -> Vector2d)? = null

    private var originOffset = Pose2d()

    private var directPose = Pose2d()

    val isInitialized: Boolean
        get() = slamera?.isStarted == true

    var poseConfidence = T265Camera.PoseConfidence.Failed
        private set

    /**
     * Current robot pose estimate.
     */
    override var poseEstimate: Pose2d
        get() {
            return directPose.transformBy(originOffset)
        }
        set(value) { originOffset = directPose.calculateTransformation(value) }

    /**
     * Current robot pose velocity
     */
    override var poseVelocity = Pose2d()

    /**
     * Completes a single localization update.
     */
    override fun update() {
        val odometryVelocity = odometryVelocityCallback?.invoke()
        if (odometryVelocity != null) {
            slamera?.sendOdometry(
                odometryVelocity.x * inToM,
                odometryVelocity.y * inToM
            )
        }
    }

    override fun cleanup() {
        slamera?.stop()
    }

    init {
        if (slamera != null) {
            slamera = T265Camera(
                Transform2d(
                    Translation2d(cameraToRobot.x * inToM, cameraToRobot.y * inToM),
                    Rotation2d(cameraToRobot.heading)
                ),
                odometryCovariance,
                hardwareMap.appContext
            )
        }
        slamera?.start()
    }

    companion object {
        const val mToIn = 2.54/100.0
        const val inToM = 100.0/2.54

        private var slamera: T265Camera? = null
    }

    /**
     * Consumes the poses passed from the camera. Automatically converts the units and types
     *  properly.
     *
     * @param update the input argument
     */
    override fun accept(update: T265Camera.CameraUpdate) {
        val rawPose = update.pose
        directPose = Pose2d(
            rawPose.x * mToIn,
            rawPose.y * mToIn,
            rawPose.heading
        )
        val rawPoseVelocity = update.velocity
        poseVelocity = Pose2d(
            rawPoseVelocity.vxMetersPerSecond * mToIn,
            rawPoseVelocity.vyMetersPerSecond * mToIn,
            rawPoseVelocity.omegaRadiansPerSecond
        )
    }
}