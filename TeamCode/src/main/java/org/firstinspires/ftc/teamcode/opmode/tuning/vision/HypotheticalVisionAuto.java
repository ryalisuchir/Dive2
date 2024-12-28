package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HangUpCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.SigmaAngleDetection;
import org.firstinspires.ftc.teamcode.common.vision.YellowAngleDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collections;
import java.util.function.BooleanSupplier;
@Autonomous
public class HypotheticalVisionAuto extends OpMode {
    Action correctionMovement;
    private RobotHardware robot;

    OpenCvWebcam webcam;

    SigmaAngleDetection sampleDetection;

    double xTravel = 0;
    double yTravel = 0;
    double angle = 0;

    KalmanFilter kalmanFilter;

    boolean isScanning = false;
    BooleanSupplier pointFoundBoolean;

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
                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION),
                        new InstantCommand(() -> {
                            isScanning = true;
                        }),
                        new WaitUntilCommand(() -> pointFoundBoolean.getAsBoolean()),
                                        new ParallelCommandGroup(
                                                new ScanningCommand(robot, mapSampleToServo(angle), Globals.EXTENDO_MAX_EXTENSION),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(1500),
                                                        new ActionCommand(correctionMovement, Collections.emptySet())
                                                )
                                )
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

        if (isScanning) {
            double greenAngle = sampleDetection.getAngleOfGreenSample();
            Point greenPoint = sampleDetection.getGreenSampleCoordinates();

            // Set the boolean supplier for detecting the sample
            pointFoundBoolean = () -> !Double.isNaN(greenAngle);

            if (!Double.isNaN(greenAngle)) {
                // Smooth the detected angle using Kalman filter
                angle = kalmanFilter.estimate((greenAngle % 170) / 180);

                // Calculate travel
                xTravel = (greenPoint.x);
                yTravel = (greenPoint.y);

                correctionMovement = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(robot.pinpointDrive.pose.position.x, robot.pinpointDrive.pose.position.y, robot.pinpointDrive.pose.heading.toDouble()))
                        .strafeToConstantHeading(new Vector2d(
                                robot.pinpointDrive.pose.position.x + xTravel,
                                robot.pinpointDrive.pose.position.y + yTravel
                        )).build();

                // Stop scanning
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

    private static final double[] SAMPLE_POSITIONS = {0.1834, 0.04, 0.205, 0.4271, 0.5531, 0.6958, 0.72, 0.8037};
    private static final double[] SERVO_POSITIONS = {0.65, 0.53, 0.65, 0.83, 0.9, 1.0, 0.45, 0.52};

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
}