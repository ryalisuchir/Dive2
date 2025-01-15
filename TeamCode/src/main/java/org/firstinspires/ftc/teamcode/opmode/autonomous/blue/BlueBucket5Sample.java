package org.firstinspires.ftc.teamcode.opmode.autonomous.blue;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.DeferredCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HangUpCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SetIntakeDownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SlideParkCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.RegularTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.RetractedTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.SlowIntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.YellowBlueDetection;
import org.firstinspires.ftc.teamcode.common.vision.YellowRedDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collections;

@Autonomous
public class BlueBucket5Sample extends OpMode {
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A, movement9A, movement10A;
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    //Vision Initialization:
    OpenCvWebcam webcam;
    YellowBlueDetection sampleDetection;
    double xTravel = 0;
    double yTravel = 0;
    double angle = 0;
    double lastEstimate;
    KalmanFilter kalmanFilter;
    boolean isScanning = false;
    boolean isResetting = false;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_SIDEWAYS_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_SIDEWAYS_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_SIDEWAYS_START_POSE)
                .splineToLinearHeading(new Pose2d(59, 58, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(54, 58, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53, 55.2, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(57, 53.5, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(64, 51.5, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(59, 56, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh() //3rd sample grab
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(61, 46.5, Math.toRadians(135)), Math.toRadians(40));

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(60, 55, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35.96, 7, Math.toRadians(0)), Math.toRadians(180.00))
                .splineToSplineHeading(
                        new Pose2d(29, 7, Math.toRadians(0)), Math.toRadians(180.00)
                );

        TrajectoryActionBuilder movement9 = movement8.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(34.56, 7, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineToLinearHeading(new Pose2d(58, 55.5, Math.toRadians(45.00)), Math.toRadians(15));

        TrajectoryActionBuilder movement10 = movement9.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35.96, 10.00, Math.toRadians(-180.00)), Math.toRadians(-180.00))
                .splineToSplineHeading(
                        new Pose2d(19, 10.00, Math.toRadians(-180.00)), Math.toRadians(-180.00),
                        new TranslationalVelConstraint(15)
                );

        movement1A = movement1.build();
        movement2A = movement2.build();
        movement3A = movement3.build();
        movement4A = movement4.build();
        movement5A = movement5.build();
        movement6A = movement6.build();
        movement7A = movement7.build();
        movement8A = movement8.build();
        movement9A = movement9.build();
        movement10A = movement10.build();

        //Vision Initialization:
        sampleDetection = new YellowBlueDetection();
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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        telemetry.addData("Ready: ", "All subsystems have been initialized!");
        telemetry.addData("Side: ", "Close");
        telemetry.addData("Description: ", "1 Specimen, 3 Basket, Park");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
//                        new HangUpCommand(robot.hangSubsystem, 1, 900),
                        new SequentialCommandGroup(
                                //First Drop:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement1A, Collections.emptySet()),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                                        ),
                                        new SetIntakeDownCommand(robot)
                                ),
                                new BucketDropCommand(robot),
                                //First Intake:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement2A, Collections.emptySet()),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1300),
                                                new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.65)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new OuttakeTransferReadyCommand(robot)
                                        )
                                ),
                                new WaitCommand(250),
                                new IntakePeckerCommand(robot),
                                new ParallelCommandGroup(
                                        new RegularTransferCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(130),
                                                new ActionCommand(movement3A, Collections.emptySet())
                                        )
                                ),
                                //Second Drop:
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                                new BucketDropCommand(robot),
                                //Second Intake:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement4A, Collections.emptySet()),
                                        new OuttakeTransferReadyCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.55)
                                        )
                                ),
                                new WaitCommand(150),
                                new IntakePeckerCommand(robot),
                                new ParallelCommandGroup(
                                        new RegularTransferCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new ActionCommand(movement5A, Collections.emptySet())
                                        )
                                ),
                                //Third Drop:
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                                new BucketDropCommand(robot),
                                new InstantCommand(() -> {isResetting = false;}),
                                //Fourth Intake:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement6A, Collections.emptySet()),
                                        new OuttakeTransferReadyCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(750),
                                                new IntakeCommand(robot, 0.75, Globals.EXTENDO_MAX_EXTENSION*0.4)
                                        )
                                ),
                                new WaitCommand(150),
                                new SlowIntakePeckerCommand(robot),
                                new ParallelCommandGroup(
                                        new RetractedTransferCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new ActionCommand(movement7A, Collections.emptySet())
                                        )
                                ),
                                //Fourth Drop:
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                                new BucketDropCommand(robot),
                                new ParallelCommandGroup(
                                        new ActionCommand(movement8A, Collections.emptySet()),
                                        new OuttakeTransferReadyCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1900),
                                                new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) Globals.EXTENDO_MAX_EXTENSION * 0.6)
                                        )
                                ),
                                //Vision stuff:
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new InstantCommand(() -> {
                                            isScanning = true;
                                        }),
                                        new WaitUntilCommand(() -> !isScanning),
                                        new ParallelCommandGroup(
                                                new DeferredCommand(() ->
                                                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
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
                                        new WaitCommand(500),
                                        new InstantCommand(() -> {
                                            isScanning = true;
                                        }),
                                        new WaitUntilCommand(() -> !isScanning),
                                        new ParallelCommandGroup(
                                                new DeferredCommand(() ->
                                                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
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
                                        new WaitCommand(500),
                                        new InstantCommand(() -> {
                                            xTravel = 0;
                                            yTravel = 0;
                                            lastEstimate = 0;
                                            isScanning = true;
                                        }),
                                        new WaitUntilCommand(() -> !isScanning),
                                        new ParallelCommandGroup(
                                                new DeferredCommand(() ->
                                                        new ScanningCommand(robot, lastEstimate, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
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
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new RegularTransferCommand(robot),
                                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new ActionCommand(movement9A, Collections.emptySet())
                                        )
                                ),
                                new BucketDropCommand(robot),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(75),
                                                new SlideParkCommand(robot)
                                        ),
                                        new ActionCommand(movement10A, Collections.emptySet())
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

        if (!isResetting) {
            robot.extendoSubsystem.extendoSlidesLoop();
        }

        telemetry.addLine("Currently running: 0+6 (0 Specimen 6 High Basket)");
        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);
        telemetry.addData("Robot Position: ", robot.pinpointDrive.pose.position);
        telemetry.addData("Extendo State: ", Globals.extendoState);
        telemetry.addData("Outtake State: ", Globals.outtakeState);
        telemetry.addData("Intake Rotation State: ", Globals.intakeRotationState);
        telemetry.addData("Intake Coaxial State: ", Globals.intakeCoaxialState);
        telemetry.addData("Intake Claw State: ", Globals.intakeClawState);
        telemetry.addData("FourBar State: ", Globals.fourBarState);
        telemetry.addData("Outtake Arm State: ", Globals.outtakeArmState);
        telemetry.addData("Outtake Claw State: ", Globals.outtakeClawState);

        if (Globals.extendoFailState == Globals.ExtendoFailState.FAILED_EXTEND) {
            Log.i("Extendo Failed:", "FAILED_EXTENSION");
        }

        if (Globals.extendoFailState == Globals.ExtendoFailState.FAILED_RETRACT) {
            Log.i("Extendo Failed:", "FAILED_RETRACTION");
        }

        if (isScanning) {
            double greenAngle = sampleDetection.getAngleOfGreenSample();
            Point greenPoint = sampleDetection.getGreenSampleCoordinates();

            if (!Double.isNaN(greenAngle)) {

                // Update travel points based on the detected point
                xTravel = greenPoint.x;
                yTravel = greenPoint.y;

                // Calculate the new estimate
                double angle = (greenAngle % 180) / 180; // Claw angle between 0 and 1

                // Update lastEstimate with the new valid estimate
                lastEstimate = angle;

                // Mark scanning as complete
                isScanning = false;
            } else {
                // Fallback to the last valid estimate and previous travel points
                angle = lastEstimate;
            }
        }

        telemetry.addData("Right Lift Pos: ", robot.rightLift.getCurrentPosition());

        loop = time;
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