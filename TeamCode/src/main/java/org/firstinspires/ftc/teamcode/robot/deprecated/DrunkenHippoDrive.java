package org.firstinspires.ftc.teamcode.robot.deprecated;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.ColorDetectionHelper;
import org.firstinspires.ftc.teamcode.robot.StandardTrackingWheelLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.robot.deprecated.HippoDriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
//simon was here
@SuppressWarnings("unused")
@Deprecated
public class DrunkenHippoDrive extends MecanumDrive {
    // VISION STUFF
    public enum RingPosition {FOUR, ONE, NONE}

    private final RingDeterminationPipeline pipeline = new RingDeterminationPipeline();

    // NOT VISION STUFF
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public static int POSE_HISTORY_LIMIT = 100;
    
    private final double FLYWHEEL_RMP_MULTIPLIER = 60.0/28.0; // Should be `<secondsPerMinute> / <ticksPerRevolution>`

    public enum Mode {IDLE, TURN, FOLLOW_TRAJECTORY}

    private final FtcDashboard dashboard;
    private final NanoClock clock;

    private Mode mode;

    private final PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private final TrajectoryFollower follower;

    private final LinkedList<Pose2d> poseHistory;

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx intake;
    private final DcMotorEx transfer;
    private final DcMotorEx wobblePivot;
    private final DcMotorEx flywheel;

    private double flywheelTargetRPM;

    private final Servo wobbleGrab;
    private final Servo shooterTrigger;
    private final Servo ringStop;

    // Servo position enumerations
    public enum RingStopPos {
        INIT (0.25),
        START (0.5),
        END (0.85);
        private final double value;
        RingStopPos(double value){this.value = value;}
    }
    public enum TriggerPos {
        INIT (0),
        START (0),
        END (0.18);
        private final double value;
        TriggerPos(double value){this.value = value;}
    }
    public enum WobbleGrabPos {
        INIT (1),
        START (1),
        END (0.5);
        private final double value;
        WobbleGrabPos(double value){this.value = value;}
    }

    private final List<DcMotorEx> motors;

    private final VoltageSensor batteryVoltageSensor;

    private final StandardTrackingWheelLocalizer localizer;

    private Pose2d lastPoseOnTurn;


    private static class RingDeterminationPipeline extends OpenCvPipeline {
        //Some color constants
        private static final Scalar BLUE = new Scalar(0, 0, 255);
        private static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions
        private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(ColorDetectionHelper.TopLeftX, ColorDetectionHelper.TopLeftY);

        private final Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        private final Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + ColorDetectionHelper.Width,
                REGION1_TOPLEFT_ANCHOR_POINT.y + ColorDetectionHelper.Height);

        /*
         * Working variables
         */
        private Mat region1_Cb;
        private final Mat YCrCb = new Mat();
        private final Mat Cb = new Mat();
        private int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile DrunkenHippoDrive.RingPosition position = DrunkenHippoDrive.RingPosition.FOUR;

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

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = DrunkenHippoDrive.RingPosition.FOUR; // Record our analysis
            if (avg1 > ColorDetectionHelper.FourRingThresh) {
                position = DrunkenHippoDrive.RingPosition.FOUR;
            } else if (avg1 > ColorDetectionHelper.OneRingThresh) {
                position = DrunkenHippoDrive.RingPosition.ONE;
            } else {
                position = DrunkenHippoDrive.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
        public DrunkenHippoDrive.RingPosition getPosition(){
            return position;
        }
    }

    public DrunkenHippoDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        //constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        org.firstinspires.ftc.teamcode.util.LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        // Shooting mechanism motors:
        intake = hardwareMap.get(DcMotorEx.class, "IntakeRoller");
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        ringStop = hardwareMap.get(Servo.class, "RingStop");

        // Wobble grabber motors
        wobbleGrab = hardwareMap.get(Servo.class, "WobbleGrab");
        shooterTrigger = hardwareMap.get(Servo.class, "ShooterTrigger");
        wobblePivot = hardwareMap.get(DcMotorEx.class, "WobblePivot");

        wobblePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setWobblePivot(0);
        setWobbleGrab(WobbleGrabPos.INIT);
        dropStop(RingStopPos.INIT);
        pressTrigger(false);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelTargetRPM = 0;

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // _TODO: reverse the flywheel by uncommenting this line if needed
        //  flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        setLocalizer(localizer);

        // Camera stuff
        OpenCvWebcam webCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "camra"),
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()));
        webCam.setPipeline(pipeline);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(() -> {
            //if the camera is open start steaming
            webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT  );
        });

        dashboard.startCameraStream(webCam, 10);
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        flywheel.setVelocity(flywheelTargetRPM / FLYWHEEL_RMP_MULTIPLIER);

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                org.firstinspires.ftc.teamcode.util.DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                org.firstinspires.ftc.teamcode.util.DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            default:
                break;
        }

        fieldOverlay.setStroke("#3F51B5");
        org.firstinspires.ftc.teamcode.util.DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void setIntakePowers(double t){
        intake.setPower(t);
        transfer.setPower(t);
    }

    public void setWobbleGrab(WobbleGrabPos pos){
        wobbleGrab.setPosition(pos.value);
    }

    public void setWobblePivot(double power){
        wobblePivot.setPower(power);
    }

    // Sets the target velocity of the flywheel
    public void revFlywheel(double rpm){
        flywheelTargetRPM = rpm;
    }

    // Gets the flywheel velocity
    public double getFlywheelVelo(){
        return flywheel.getVelocity() * FLYWHEEL_RMP_MULTIPLIER;
    }

    // Gets the difference between the target flywheel velocity and the actual flywheel velocity
    public double getFlywheelVeloDiff (){
        return flywheel.getVelocity() * FLYWHEEL_RMP_MULTIPLIER - flywheelTargetRPM;
    }

    @SuppressWarnings("StatementWithEmptyBody")
    public void waitForFlywheel(double threshold){
        while (Math.abs(getFlywheelVeloDiff()) < threshold);
    }
    public void pressTrigger(boolean enabled){
        if (enabled){
            shooterTrigger.setPosition(TriggerPos.END.value);
        } else {
            shooterTrigger.setPosition(TriggerPos.START.value);
        }
    }

    public void dropStop(RingStopPos pos){
        ringStop.setPosition(pos.value);
    }

    public void cancelTrajectory () {
        mode = Mode.IDLE;
    }

    public int getAnalysis()
    {
        return pipeline.getAnalysis();
    }
    public DrunkenHippoDrive.RingPosition getPosition(){
        return pipeline.getPosition();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
    }
}
