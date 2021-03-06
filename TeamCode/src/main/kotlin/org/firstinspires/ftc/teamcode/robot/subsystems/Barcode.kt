package org.firstinspires.ftc.teamcode.robot.subsystems

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Cameras
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.BarcodeConstants
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Subsystem to manage the camera / barcode reading.
 *
 * @constructor
 * Initializes the camera with an EOCV pipeline for barcode reading.
 *
 * @param hardwareMap Passed in from [org.firstinspires.ftc.teamcode.robot.CheckmateRobot]
 *
 * @see org.firstinspires.ftc.teamcode.robot.subsystems.barcode
 */
class Barcode(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "Barcode"
    override val subsystems = SubsystemMap{ tag }

    private val webcam: OpenCvCamera

    /**
     * Whether the camera is streaming to the pipeline.
     */
    var isStreaming: Boolean? = null
        private set

    // Closes the camera
    override fun preLoop() {
        if (isStreaming == true) {
            webcam.stopStreaming()
            isStreaming = false
        } else if (isStreaming == null) {
            isStreaming = false
        }
    }

    /**
     * Detected position of the TSE on the barcode.
     *
     * @see BarcodePosition
     */
    val position: BarcodePosition
        get() = pipeline.position

    /**
     * Raw analysis of the two read barcode positions.
     */
    val analysis: List<Int>
        get() = pipeline.analysis

    /**
     * Represents the position of the TSE on the barcode.
     */
    enum class BarcodePosition {
        LEFT, MIDDLE, RIGHT;

        /**
         * Get the corresponding [Lift.Points] entry.
         *
         * @return The corresponding [Lift.Points] entry.
         */
        fun toLiftPoint(): Lift.Points {
            return when(this) {
                LEFT -> Lift.Points.HIGH
                MIDDLE -> Lift.Points.MID
                RIGHT -> Lift.Points.LOW
            }
        }
    }

    private val pipeline = BarcodeDeterminationPipeline()

    private class BarcodeDeterminationPipeline : OpenCvPipeline() {
        /*
         * Working variables
         * cb is the used to isolate the blue in the feed.
         */
        private var leftBoxY: Mat? = null
        private var middleBoxY: Mat? = null
        private val yCrCb = Mat()
        private val y = Mat()
        private var leftValue = 0
        private var middleValue = 0

        // Volatile since accessed by OpMode thread w/o synchronization
        @Volatile
        var position = BarcodePosition.LEFT
            private set

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        private fun inputToCb(input: Mat) {
            Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb)
            Core.extractChannel(yCrCb, y, 0)
        }

        override fun init(firstFrame: Mat) {
            inputToCb(firstFrame)

            // LEFTBOX_Cb and MIDDLEBOX_Cb are the blue content of their respective boxes.
            leftBoxY = y.submat(BarcodeConstants.leftBox.rectangle)
            middleBoxY = y.submat(BarcodeConstants.middleBox.rectangle)
        }

        override fun processFrame(input: Mat): Mat {
            inputToCb(input)

            // 
            leftValue = Core.mean(leftBoxY).`val`[0].toInt()
            middleValue = Core.mean(middleBoxY).`val`[0].toInt()
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.leftBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.leftBox.pointB,  // Second point which defines the rectangle
                PURPLE,  // The color the rectangle is drawn in
                2
            ) // Thickness of the rectangle lines
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.middleBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.middleBox.pointB,  // Second point which defines the rectangle
                PURPLE,  // The color the rectangle is drawn in
                2
            ) // Thickness of the rectangle lines
            position = BarcodePosition.LEFT // Record our analysis
            position = if (leftValue > middleValue && leftValue > BarcodeConstants.leftBox.threshold) {
                BarcodePosition.LEFT
            } else if (middleValue > BarcodeConstants.middleBox.threshold) {
                BarcodePosition.MIDDLE
            } else {
                BarcodePosition.RIGHT
            }
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.leftBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.leftBox.pointB,  // Second point which defines the rectangle
                if (position == BarcodePosition.LEFT) YELLOW else RED,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
            Imgproc.rectangle(
                input,  // Buffer to draw on
                BarcodeConstants.middleBox.pointA,  // First point which defines the rectangle
                BarcodeConstants.middleBox.pointB,  // Second point which defines the rectangle
                if (position == BarcodePosition.MIDDLE) YELLOW else BLUE,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
            return input
        }

        val analysis: List<Int>
            get() = listOf(leftValue, middleValue)

        companion object {
            // Some color constants
            // These dictate the color of the boxes when you see the camera output
            private val RED = Scalar(255.0, 0.0, 0.0)
            private val BLUE = Scalar(0.0, 0.0, 255.0)
            private val PURPLE = Scalar(51.0, 12.0, 47.0)
            private val YELLOW = Scalar(255.0, 255.0, 0.0)
        }
    }

    init {
        // Instantiates the webcam "webcam" for OpenCv to use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            Cameras.WEBCAM.get(hardwareMap),
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.packageName
            )
        )
        webcam.setPipeline(pipeline)

        // listens for when the camera is opened
        webcam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                if (isStreaming == null) {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN)
                    isStreaming = true
                    // Streams camera output to the FTCDashboard
                    FtcDashboard.getInstance().startCameraStream(webcam, 12.0)
                }
            }

            override fun onError(errorCode: Int) {
                Log.e(tag, "OpenCV threw error code $errorCode")
            }
        })
    }
}