package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers

import android.os.SystemClock.sleep
import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.spartronics4915.lib.T265Camera
import org.apache.commons.math3.stat.correlation.Covariance
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemContext
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer.Companion.inToM
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.T265Localizer.Companion.mToIn
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.SuperTrajectorySequenceRunner.Drawable
import java.util.function.Consumer

/**
 * Convert a RoadRunner Pose2d to an FTCLib Pose2d and convert inches to meters.
 *
 * @return An FTCLib Pose2d in meters.
 */
fun Pose2d.toFtcLib(): com.arcrobotics.ftclib.geometry.Pose2d {
    return com.arcrobotics.ftclib.geometry.Pose2d(
        this.x * inToM,
        this.y * inToM,
        Rotation2d(this.heading)
    )
}

/**
 * Convert a FTCLib Pose2d to a RoadRunner Pose2d and convert meters to inches.
 *
 * @return A RoadRunner Pose2d in inches.
 */
fun com.arcrobotics.ftclib.geometry.Pose2d.toRoadRunner(): Pose2d {
    return Pose2d(
        this.x * mToIn,
        this.y * mToIn,
        this.heading
    )
}

/**
 * Convert a FTCLib ChassisSpeeds to a RoadRunner Pose2d and convert meters to inches.
 *
 * @return A RoadRunner Pose2d in inches.
 */
fun ChassisSpeeds.toRoadRunner(): Pose2d {
    return Pose2d(
        this.vxMetersPerSecond * mToIn,
        this.vyMetersPerSecond * mToIn,
        this.omegaRadiansPerSecond
    )
}

class T265Localizer(
    context: SubsystemContext,
    private var cameraToRobot: Pose2d,
    private var odometryCovariance: Double
): Localizer, Consumer<T265Camera.CameraUpdate>, AbstractSubsystem {
    override val tag = "T265Localizer"
    override val subsystems = SubsystemMap{ tag }

    private val slamera: T265Camera

    // LOCKS //
    private object UpdateMutex

    // SLAMERA STUFF //
    @Volatile private var updatesReceived = 0

    fun waitForUpdate() {
        val lastUpdatesReceived = updatesReceived
        while (lastUpdatesReceived == updatesReceived) { continue }
    }

    var odometryVelocityCallback: (() -> Vector2d)? = null

    private var lastUpdate: AmericanCameraUpdate = AmericanCameraUpdate()
        get() {
            synchronized(UpdateMutex) {
                return field
            }
        }
        set(value) {
            synchronized(UpdateMutex) {
                field = value
            }
        }

    val poseConfidence: T265Camera.PoseConfidence
        get() {
            return lastUpdate.confidence
        }

    /**
     * Current robot pose estimate.
     */
    override var poseEstimate: Pose2d
        get() {
            waitForUpdate()
            return lastUpdate.pose
        }
        set(value) {
            lastUpdate.pose = value
            val temp = value.toFtcLib()
            slamera.setPose(
                com.arcrobotics.ftclib.geometry.Pose2d(
                    temp.translation.rotateBy(Rotation2d(cameraToRobot.heading)),
                    temp.rotation
                )
            )
        }

    /**
     * Current robot pose velocity
     */
    override val poseVelocity: Pose2d? = null

    /**
     * Sets the odometry info of the camera.
     *
     * @param offset The offset from the camera's perspective to the robot.
     * @param covariance Changes the ratio of trust between the camera's sensor and the passed in odometry data
     */
    fun setOdometryInfo(offset: Pose2d, covariance: Double = odometryCovariance) {
        val ftcLibCameraToRobot = cameraToRobot.toFtcLib()
        cameraToRobot = offset
        if (covariance != odometryCovariance) {
            odometryCovariance = covariance
        }/*
        slamera.setOdometryInfo(
            Transform2d(
                ftcLibCameraToRobot.translation,
                ftcLibCameraToRobot.rotation
            ),
            covariance
        )*/
    }

    /**
     * Completes a single localization update.
     */
    override fun update() {
        val odometryVelocity = odometryVelocityCallback?.invoke()
        if (odometryVelocity != null) {
            slamera.sendOdometry(
                odometryVelocity.x * inToM,
                odometryVelocity.y * inToM
            )
        }
    }

    override fun cleanup() {
        slamera.stop()
    }

    init {
        Log.d(tag, "Initializing T265")
        if (persistentSlamera == null) {
            Log.d(tag, "Slamera was null.")
            val ftcLibCameraToRobot = cameraToRobot.toFtcLib()
            persistentSlamera = T265Camera(
                Transform2d(
                    ftcLibCameraToRobot.translation,
                    ftcLibCameraToRobot.rotation
                ),
                odometryCovariance,
                context.hardwareMap.appContext
            )
        }
        slamera = persistentSlamera!!
        sleep(1000)
        if (!slamera.isStarted) {
            Log.i(tag, "Starting camera...")
            slamera.start(this)
        } else {
            Log.i(tag, "Camera was already started.")
        }
        logPose(poseEstimate)
        poseEstimate = Pose2d()
        logPose(poseEstimate)

        Thread {
            while (context.subsystems["Drivetrain"] == null) {continue}
            Log.i(tag, "Setting up dashboard telemetry.")
            val drivetrain = context.subsystems["Drivetrain"]!! as Drivetrain
            drivetrain.dashTelemetry["rawX"] = {lastUpdate.directPose.x}
            drivetrain.dashTelemetry["rawY"] = {lastUpdate.directPose.y}
            drivetrain.dashTelemetry["rawHeading (deg)"] = {Math.toDegrees(lastUpdate.directPose.heading)}
            drivetrain.dashTelemetry["poseConfidence"] = {lastUpdate.confidence.ordinal}
            //drivetrain.drawnTelemetry.add(Drawable("#ffff00") { lastUpdate.directPose })
        }.start()
    }

    companion object {
        const val mToIn = 100.0/2.54
        const val inToM = 2.54/100.0

        // TODO: add logging for the direct values of the camera
        private var persistentSlamera: T265Camera? = null
    }

    /**
     * Consumes the poses passed from the camera. Automatically converts the units and types
     *  properly.
     *
     * @param update the input argument
     */
    override fun accept(update: T265Camera.CameraUpdate) {
        updatesReceived++
        synchronized(UpdateMutex) {
            lastUpdate = AmericanCameraUpdate(update, cameraToRobot.heading)
        }
    }

    private fun logPose(pose: Pose2d) {
        Log.d(tag, "${pose.x}, ${pose.y}, ${pose.heading}")
    }

    /**
     * CameraUpdate alternative that speaks in hamburgers, football fields, and guns, rather than
     *  tea and crumpets
     *
     * @param update Your old, nasty, Bri'ish, tea-and-crumpets-speaking CameraUpdate object.
     */
    class AmericanCameraUpdate(
        update: T265Camera.CameraUpdate = T265Camera.CameraUpdate(
            com.arcrobotics.ftclib.geometry.Pose2d(),
            ChassisSpeeds(),
            T265Camera.PoseConfidence.Failed
        ),
        extraAngle: Double = 0.0
    ) {
        private var extraRotation = Rotation2d(-extraAngle)

        // TODO: apply some sort of filtering
        var pose = com.arcrobotics.ftclib.geometry.Pose2d(
            update.pose.translation.rotateBy(extraRotation),
            update.pose.rotation.rotateBy(extraRotation)
        ).toRoadRunner()
            set(value) {
                if (confidence == T265Camera.PoseConfidence.Failed) {
                    throw BadPoseSetException()
                }
                field = value
                velocity = Pose2d()
                confidence = T265Camera.PoseConfidence.High
            }
        var velocity = update.velocity.toRoadRunner()
            private set
        var confidence: T265Camera.PoseConfidence = update.confidence
            private set
        val directPose = update.pose.toRoadRunner()
    }

    class BadPoseSetException: Exception("Attempted to set pose while confidence was Failed.")
}