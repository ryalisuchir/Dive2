package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.DeferredCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.RegularTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.SigmaAngleDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collections;

@Autonomous
public class HypotheticalVisionAuto extends OpMode {
    private static final double[] SAMPLE_POSITIONS = {0.1834, 0, 0.205, 0.4271, 0.5531, 0.6958, 0.72, 0.8037, 1};
    private static final double[] SERVO_POSITIONS = {0.65, 0.55, 0.65, 0.83, 0.9, 1.0, 0.45, 0.52, 0.55};
    OpenCvWebcam webcam;
    SigmaAngleDetection sampleDetection;
    double xTravel = 0;
    double yTravel = 0;
    double angle = 0;
    KalmanFilter kalmanFilter;
    boolean isScanning = false;
    private RobotHardware robot;

    public static double mapSampleToServo(double samplePosition) {
        // Check bounds
        if (samplePosition <= SAMPLE_POSITIONS[0]) {
            return SERVO_POSITIONS[0];
        }
        if (samplePosition >= SAMPLE_POSITIONS[SAMPLE_POSITIONS.length - 1]) {
            return SERVO_POSITIONS[SERVO_POSITIONS.length - 1];
        }

        for (int i = 0; i < SAMPLE_POSITIONS.length - 1; i++) {
            if (samplePosition >= SAMPLE_POSITIONS[i] && samplePosition <= SAMPLE_POSITIONS[i + 1]) {
                double t = (samplePosition - SAMPLE_POSITIONS[i]) / (SAMPLE_POSITIONS[i + 1] - SAMPLE_POSITIONS[i]);
                return SERVO_POSITIONS[i] + t * (SERVO_POSITIONS[i + 1] - SERVO_POSITIONS[i]);
            }
        }

        return -1;
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        sampleDetection = new SigmaAngleDetection();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(sampleDetection);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error: ", errorCode);
            }
        });

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.DEFAULT_START_POSE);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        // Initialize Kalman filter with sample parameters
        double Q = 10; // Process noise covariance
        double R = 2;  // Measurement noise covariance
        int N = 3;     // Number of historical estimates
        kalmanFilter = new KalmanFilter(Q, R, N);
        telemetry.addLine("Finished setting up auto.");
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        telemetry.addLine("Running: HypotheticalVisionAuto");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) Globals.EXTENDO_MAX_EXTENSION / 2),
                        new WaitCommand(1000),
                        new InstantCommand(() -> {
                            isScanning = true;
                        }),
                        new WaitUntilCommand(() -> !isScanning),
                        new ParallelCommandGroup(
                                new DeferredCommand(() ->
                                        new ActionCommand(
                                                robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
                                                        .strafeToConstantHeading(new Vector2d(
                                                                robot.driveSubsystem.getPoseEstimate().position.x + yTravel,
                                                                robot.driveSubsystem.getPoseEstimate().position.y + xTravel
                                                        )).build()
                                                , Collections.emptySet())
                                        , robot.driveSubsystem)
                        ),
                        new InstantCommand(() -> {
                            isScanning = true;
                        }),
                        new WaitUntilCommand(() -> !isScanning),
                        new ParallelCommandGroup(
                                new ScanningCommand(robot, mapSampleToServo(angle), (double) Globals.EXTENDO_MAX_EXTENSION / 2),
                                new DeferredCommand(() ->
                                        new ActionCommand(
                                                robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
                                                        .strafeToConstantHeading(new Vector2d(
                                                                robot.driveSubsystem.getPoseEstimate().position.x + yTravel,
                                                                robot.driveSubsystem.getPoseEstimate().position.y + xTravel
                                                        )).build()
                                                , Collections.emptySet())
                                        , robot.driveSubsystem)
                        ),
                        new IntakePeckerCommand(robot),
                        new WaitCommand(100),
                        new RegularTransferCommand(robot)
                )
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.depositSubsystem.outtakeSlidesLoop();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();

        telemetry.addData("xTravel: ", xTravel);
        telemetry.addData("yTravel: ", yTravel);
        telemetry.addData("Angle of Sample Detected: ", angle);


        if (isScanning) {
            double greenAngle = sampleDetection.getAngleOfGreenSample();
            Point greenPoint = sampleDetection.getGreenSampleCoordinates();

            if (!Double.isNaN(greenAngle)) {
                // Smooth the detected angle using Kalman filter
                angle = kalmanFilter.estimate((greenAngle % 180) / 180);

                xTravel = (greenPoint.x) - 1;
                yTravel = -(greenPoint.y) - 1;

                isScanning = false;
            }

        }

        telemetry.update();
        robot.clearCache();
    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}