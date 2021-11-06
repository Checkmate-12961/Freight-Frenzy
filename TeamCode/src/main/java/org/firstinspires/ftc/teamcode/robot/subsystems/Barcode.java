package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.BarcodeConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class Barcode implements AbstractSubsystem {
    public Barcode(HardwareMap hardwareMap) {
        OpenCvWebcam webCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, HardwareNames.Cameras.WEBCAM.name),
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()));
        webCam.setPipeline(pipeline);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard.getInstance().startCameraStream(webCam, 12);
    }

    @Override
    public void update() {
        // nothing to update
    }

    @Override
    public void cleanup() {
        // nothing to clean up
    }

    public enum BarcodePosition {LEFT, MIDDLE, RIGHT}

    private final RingDeterminationPipeline pipeline = new RingDeterminationPipeline();


    private static class RingDeterminationPipeline extends OpenCvPipeline {
        //Some color constants
        //These dictate the color of the boxes when you see the camera output
        private static final Scalar RED = new Scalar(255, 0, 0);
        private static final Scalar YELLOW = new Scalar(255, 255, 0);
        private static final Scalar BLUE = new Scalar(0, 0, 255);
        private static final Scalar PURPLE = new Scalar(51, 12, 47);

        //The core values which define the location and size of the sample regions
        private final Point LEFTBOX_TOPLEFT_ANCHOR_POINT = new Point(BarcodeConstants.LeftBoxX, BarcodeConstants.LeftBoxY);

        private final Point LEFTBOX_pointA = new Point(
                LEFTBOX_TOPLEFT_ANCHOR_POINT.x,
                LEFTBOX_TOPLEFT_ANCHOR_POINT.y);
        private final Point LEFTBOX_pointB = new Point(
                LEFTBOX_TOPLEFT_ANCHOR_POINT.x + BarcodeConstants.LeftBoxWidth,
                LEFTBOX_TOPLEFT_ANCHOR_POINT.y + BarcodeConstants.LeftBoxHeight);

        private final Point MIDDLEBOX_TOPLEFT_ANCHOR_POINT = new Point(BarcodeConstants.MiddleBoxX, BarcodeConstants.MiddleBoxY);

        private final Point MIDDLEBOX_pointA = new Point(
                MIDDLEBOX_TOPLEFT_ANCHOR_POINT.x,
                MIDDLEBOX_TOPLEFT_ANCHOR_POINT.y);
        private final Point MIDDLEBOX_pointB = new Point(
                MIDDLEBOX_TOPLEFT_ANCHOR_POINT.x + BarcodeConstants.MiddleBoxWidth,
                MIDDLEBOX_TOPLEFT_ANCHOR_POINT.y + BarcodeConstants.MiddleBoxHeight);

        private final Point RIGHTBOX_TOPLEFT_ANCHOR_POINT = new Point(BarcodeConstants.RightBoxX, BarcodeConstants.RightBoxY);

        private final Point RIGHTBOX_pointA = new Point(
                RIGHTBOX_TOPLEFT_ANCHOR_POINT.x,
                RIGHTBOX_TOPLEFT_ANCHOR_POINT.y);
        private final Point RIGHTBOX_pointB = new Point(
                RIGHTBOX_TOPLEFT_ANCHOR_POINT.x + BarcodeConstants.RightBoxWidth,
                RIGHTBOX_TOPLEFT_ANCHOR_POINT.y + BarcodeConstants.RightBoxHeight);
        /*
         * Working variables
         */
        private Mat LEFTBOX_Cb;
        private Mat MIDDLEBOX_Cb;
        private Mat RIGHTBOX_Cb;
        private final Mat YCrCb = new Mat();
        private final Mat Cb = new Mat();
        private int leftValue;
        private int middleValue;
        private int rightValue;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile BarcodePosition position = BarcodePosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            LEFTBOX_Cb = Cb.submat(new Rect(LEFTBOX_pointA, LEFTBOX_pointB));
            MIDDLEBOX_Cb = Cb.submat(new Rect(MIDDLEBOX_pointA, MIDDLEBOX_pointB));
            RIGHTBOX_Cb = Cb.submat(new Rect(RIGHTBOX_pointA, RIGHTBOX_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            leftValue = (int) Core.mean(LEFTBOX_Cb).val[0];
            middleValue = (int) Core.mean(MIDDLEBOX_Cb).val[0];
            rightValue = (int) Core.mean(RIGHTBOX_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    LEFTBOX_pointA, // First point which defines the rectangle
                    LEFTBOX_pointB, // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    MIDDLEBOX_pointA, // First point which defines the rectangle
                    MIDDLEBOX_pointB, // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    LEFTBOX_pointA, // First point which defines the rectangle
                    LEFTBOX_pointB, // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            position = BarcodePosition.LEFT; // Record our analysis
            if ((leftValue>middleValue) && (leftValue>rightValue)) {
                position = BarcodePosition.LEFT;
            } else if (middleValue>rightValue) {
                position = BarcodePosition.MIDDLE;
            } else {
                position = BarcodePosition.RIGHT;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    LEFTBOX_pointA, // First point which defines the rectangle
                    LEFTBOX_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    MIDDLEBOX_pointA, // First point which defines the rectangle
                    MIDDLEBOX_pointB, // Second point which defines the rectangle
                    YELLOW, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RIGHTBOX_pointA, // First point which defines the rectangle
                    RIGHTBOX_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return leftValue;
        }

        public BarcodePosition getPosition() {
            return position;
        }
    }
}