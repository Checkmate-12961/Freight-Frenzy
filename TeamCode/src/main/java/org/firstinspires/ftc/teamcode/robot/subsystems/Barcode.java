package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.BarcodeConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;

public class Barcode implements AbstractSubsystem {
    private final OpenCvCamera webcam;

    public Barcode(HardwareMap hardwareMap) {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, HardwareNames.Cameras.WEBCAM.name),
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()));
        webcam.setPipeline(pipeline);

        //listens for when the camera is opened
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 12);
    }

    public BarcodePosition getPosition() {
        return pipeline.getPosition();
    }

    public List<Integer> getAnalysis() {
        return pipeline.getAnalysis();
    }

    @Override
    public void update() {
        // nothing to update
    }

    @Override
    public void cleanup() {
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        FtcDashboard.getInstance().stopCameraStream();
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

            LEFTBOX_Cb = Cb.submat(BarcodeConstants.getLeftBox());
            MIDDLEBOX_Cb = Cb.submat(BarcodeConstants.getMiddleBox());
            RIGHTBOX_Cb = Cb.submat(BarcodeConstants.getRightBox());
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            leftValue = (int) Core.mean(LEFTBOX_Cb).val[0];
            middleValue = (int) Core.mean(MIDDLEBOX_Cb).val[0];
            rightValue = (int) Core.mean(RIGHTBOX_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    BarcodeConstants.getLeftBoxPointA(), // First point which defines the rectangle
                    BarcodeConstants.getLeftBoxPointB(), // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    BarcodeConstants.getMiddleBoxPointA(), // First point which defines the rectangle
                    BarcodeConstants.getMiddleBoxPointB(), // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    BarcodeConstants.getRightBoxPointA(), // First point which defines the rectangle
                    BarcodeConstants.getRightBoxPointB(), // Second point which defines the rectangle
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
                    BarcodeConstants.getLeftBoxPointA(), // First point which defines the rectangle
                    BarcodeConstants.getLeftBoxPointB(), // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    BarcodeConstants.getMiddleBoxPointA(), // First point which defines the rectangle
                    BarcodeConstants.getMiddleBoxPointB(), // Second point which defines the rectangle
                    YELLOW, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    BarcodeConstants.getRightBoxPointA(), // First point which defines the rectangle
                    BarcodeConstants.getRightBoxPointB(), // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public List<Integer> getAnalysis() {
            return Arrays.asList(leftValue, middleValue, rightValue);
        }

        public BarcodePosition getPosition() {
            return position;
        }
    }
}