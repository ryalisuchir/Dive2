package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
    public static double xClawPosition = 320;
    public static double yClawPosition = 300;

    public static double xAxisLengthInInches = 12;
    public static double yAxisLengthInInches = 12;

    double xAxisBooster = xAxisLengthInInches / 640;
    double yAxisBooster = yAxisLengthInInches / 360;

    YellowAngleDetection sampleDetection;

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

        sampleDetection = new YellowAngleDetection();
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
                    new InstantCommand(() -> isScanning = true),
                    new WaitUntilCommand(pointFoundBoolean),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> {
                                correctionMovement = robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
                                        .strafeToConstantHeading(new Vector2d(robot.pinpointDrive.pose.position.x + xTravel, robot.pinpointDrive.pose.position.y + yTravel))
                                        .build();
                            }),
                            new ScanningCommand(robot, angle, Globals.EXTENDO_MAX_EXTENSION),
                            new IntakePeckerCommand(robot)
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
            pointFoundBoolean = () -> !Double.isNaN(greenAngle);
            if (!Double.isNaN(greenAngle)) {
                // Smooth the detected angle using Kalman filter
                angle = kalmanFilter.estimate((greenAngle % 170) / 180);

                // Calculate travel
                xTravel = (greenPoint.x - xClawPosition) * xAxisBooster;
                yTravel = -(greenPoint.y - yClawPosition) * yAxisBooster;

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

}