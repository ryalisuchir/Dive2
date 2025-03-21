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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.DeferredCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.YellowRedDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collections;

@Autonomous
@Disabled
public class HypotheticalVisionAuto extends OpMode {
    OpenCvWebcam webcam;
    YellowRedDetection sampleDetection;
    double xTravel = 0;
    double yTravel = 0;
    double estimate = 0;
    KalmanFilter kalmanFilter;
    boolean isScanning = false;
    private double lastEstimate = 0; // Preserved last valid estimate
    private org.firstinspires.ftc.teamcode.common.hardware.RobotHardware robot;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        sampleDetection = new YellowRedDetection();
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
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            isScanning = true;
                        }),
                        new WaitUntilCommand(() -> !isScanning),
                        new ParallelCommandGroup(
                                new DeferredCommand(() ->
                                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) Globals.EXTENDO_MAX_EXTENSION / 2 - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
                                        robot.extendoSubsystem),
                                new DeferredCommand(() ->
                                        new ActionCommand(
                                                robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
                                                        .strafeToConstantHeading(new Vector2d(
                                                                robot.driveSubsystem.getPoseEstimate().position.x,
                                                                robot.driveSubsystem.getPoseEstimate().position.y + xTravel
                                                        )).build()
                                                , Collections.emptySet())
                                        , robot.driveSubsystem)
                        ),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            isScanning = true;
                        }),
                        new WaitUntilCommand(() -> !isScanning),
                        new ParallelCommandGroup(
                                new DeferredCommand(() ->
                                        new IntakeCommand(robot, lastEstimate, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
                                        robot.extendoSubsystem
                                ),
                                new DeferredCommand(() ->
                                        new ActionCommand(
                                                robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
                                                        .strafeToConstantHeading(new Vector2d(
                                                                robot.driveSubsystem.getPoseEstimate().position.x,
                                                                robot.driveSubsystem.getPoseEstimate().position.y + xTravel
                                                        )).build()
                                                , Collections.emptySet())
                                        , robot.driveSubsystem)
                        ),
                        new IntakePeckerCommand(robot)
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
        telemetry.addData("Angle of Sample Detected: ", estimate);

        // Ensure consistent scanning behavior
        if (isScanning) {
            double greenAngle = sampleDetection.getAngleOfGreenSample();
            Point greenPoint = sampleDetection.getGreenSampleCoordinates();

            if (!Double.isNaN(greenAngle)) {

                // Update travel points based on the detected point


                xTravel = greenPoint.x;
                yTravel = greenPoint.y;

                // Calculate the new estimate
                estimate = (greenAngle % 180) / 180; // Claw angle between 0 and 1

                // Update lastEstimate with the new valid estimate
                lastEstimate = estimate;

                // Mark scanning as complete
                isScanning = false;
            } else {
                // Fallback to the last valid estimate and previous travel points
                estimate = lastEstimate;
            }
        }

        telemetry.addData("Final Estimate: ", lastEstimate);
        telemetry.addData("Mapped Servo Position: ", lastEstimate);
        telemetry.addData("Last Detected Green Point: ", lastEstimate);

        telemetry.update();
    }


    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}