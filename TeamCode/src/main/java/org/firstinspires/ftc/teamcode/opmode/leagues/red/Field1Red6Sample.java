//package org.firstinspires.ftc.teamcode.opmode.leagues.red;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.RegularTransferCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.ActionCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.AllSystemInitializeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.DeferredCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.SetIntakeDownCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.CameraScanningPositionCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.BucketDropCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeTransferReadyCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.RetractedTransferCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.IntakePeckerCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.SlowIntakePeckerCommand;
//import org.firstinspires.ftc.teamcode.common.hardware.Globals;
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.hardware.ZoneLookupTable;
//import org.firstinspires.ftc.teamcode.common.vision.YellowRedDetection;
//import org.opencv.core.Point;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.Collections;
//
//@Autonomous
//public class Field1Red6Sample extends OpMode { //may veer bless us
//    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A, movement9A, movement8B, movement9B, movement10A;
//    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7, movement8, movement9, movement82, movement92, movement10;
//    //Vision Initialization:
//    OpenCvWebcam webcam;
//    YellowRedDetection sampleDetection;
//    double xTravel = 0;
//    double yTravel = 0;
//    double angle = 0;
//    double lastEstimate;
//    boolean isScanning = false;
//    private RobotHardware robot;
//    private ElapsedTime time_since_start;
//    private double loop;
//
//    //Human Input for Vision:
//    ZoneLookupTable lookupTable;
//    private int submersibleFirstZone = 1;
//    private int submersibleSecondZone = 1;
//    private boolean isCycleOneSelected = true;
//    private boolean isLocked = false;
//    private long lastBlinkTime = 0;
//    private boolean showNumber = true;
//
//    private boolean prevDpadUp = false;
//    private boolean prevDpadDown = false;
//    private boolean prevDpadLeft = false;
//    private boolean prevDpadRight = false;
//    private boolean prevCross = false;
//
//
//    @Override
//    public void init() {
//        CommandScheduler.getInstance().reset();
//        lookupTable = new ZoneLookupTable();
//
//        robot = new RobotHardware(hardwareMap, Globals.BLUE_SIDEWAYS_START_POSE, true);
//
//        telemetry.addData("Ready: ", "Initialized subsystems.");
//        telemetry.update();
//
//        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
//        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_SIDEWAYS_START_POSE);
//
//        movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_SIDEWAYS_START_POSE)
//                .splineToLinearHeading(
//                        new Pose2d(58, 59, Math.toRadians(45)), Math.toRadians(45),
//                        null,
//                        new ProfileAccelConstraint(-85, 85)
//                );
//
//        movement2 = movement1.endTrajectory().fresh()
//                .setReversed(true)
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(54.2, 55, Math.toRadians(90)), Math.toRadians(0));
//
//        movement3 = movement2.endTrajectory().fresh()
//                .setReversed(false)
//                .splineToLinearHeading(
//                        new Pose2d(59, 56, Math.toRadians(45)), Math.toRadians(45));
//
//        movement4 = movement3.endTrajectory().fresh()
//                .setReversed(true)
//                .splineToLinearHeading(
//                        new Pose2d(63.6, 51, Math.toRadians(90)), Math.toRadians(90));
//
//        movement5 = movement4.endTrajectory().fresh()
//                .setReversed(false)
//                .splineToLinearHeading(
//                        new Pose2d(62, 59, Math.toRadians(45)), Math.toRadians(45));
//
//        movement6 = movement5.endTrajectory().fresh() //3rd sample grab
//                .setReversed(true)
//                .splineToLinearHeading(
//                        new Pose2d(62.5, 46.5, Math.toRadians(135)), Math.toRadians(40));
//
//        movement7 = movement6.endTrajectory().fresh()
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(61, 57, Math.toRadians(45)), Math.toRadians(45));
//
//        movement9 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(27, 7, Math.toRadians(0)))
//                .setReversed(false)
//                .splineTo(
//                        new Vector2d(56, 53), Math.toRadians(45.00)
//                );
//
//        movement92 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(27, 3, Math.toRadians(0)))
//                .setReversed(false)
//                .splineTo(
//                        new Vector2d(59, 58), Math.toRadians(45.00),
//                        null,
//                        new ProfileAccelConstraint(-30, 85)
//                );
//
//        movement10 = movement92.endTrajectory().fresh()
//                .setReversed(false)
//                .strafeToConstantHeading(new Vector2d(56, 54), null, new ProfileAccelConstraint(-85, 85));
//
//        movement1A = movement1.build();
//        movement2A = movement2.build();
//        movement3A = movement3.build();
//        movement4A = movement4.build();
//        movement5A = movement5.build();
//        movement6A = movement6.build();
//        movement7A = movement7.build();
//        movement9A = movement9.build();
//        movement9B = movement92.build();
//        movement10A = movement10.build();
//
//        //Vision Initialization:
//        sampleDetection = new YellowRedDetection();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//        webcam.setPipeline(sampleDetection);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error: ", errorCode);
//            }
//        });
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(webcam, 60);
//    }
//
//    @Override
//    public void init_loop() {
//        robot.clearCache();
//        if (isLocked) {
//            telemetry.addData("Ready: ", "All subsystems have been initialized!");
//            telemetry.addData("Side: ", "Close");
//            telemetry.addData("Description: ", "6 Samples");
//            telemetry.addLine("If a cycle decision must be changed, please restart the OpMode.");
//            telemetry.addData("Submersible First Cycle: ", submersibleFirstZone);
//            telemetry.addData("Submersible Second Cycle: ", submersibleSecondZone);
//        }
//
//        // Rising edge detection for the cross button
//        if (gamepad1.cross && !prevCross) {
//            isLocked = true; // Lock the selection only on the rising edge
//        }
//
//        if (!isLocked) {
//            telemetry.addLine("Selection to be made:");
//            telemetry.addLine("Click X to lock the selection.");
//
//            telemetry.addData("Cycle 1: ", (isCycleOneSelected && !showNumber && !isLocked ? " " : submersibleFirstZone));
//            telemetry.addData("Cycle 2: ", (!isCycleOneSelected && !showNumber && !isLocked ? " " : submersibleSecondZone));
//
//            // Rising edge detection for dpad_left and dpad_right
//            if (gamepad1.dpad_left && !prevDpadLeft) {
//                isCycleOneSelected = !isCycleOneSelected;
//            }
//            if (gamepad1.dpad_right && !prevDpadRight) {
//                isCycleOneSelected = !isCycleOneSelected;
//            }
//
//            // Rising edge detection for dpad_up and dpad_down
//            if (gamepad1.dpad_up && !prevDpadUp) {
//                if (isCycleOneSelected) {
//                    submersibleFirstZone = (submersibleFirstZone % 4) + 1;
//                }
//            }
//            if (gamepad1.dpad_down && !prevDpadDown) {
//                if (isCycleOneSelected) {
//                    submersibleFirstZone = (submersibleFirstZone == 1 ? 4 : submersibleFirstZone - 1);
//                } else {
//                    submersibleSecondZone = (submersibleSecondZone == 1 ? 4 : submersibleSecondZone - 1);
//                }
//            }
//        }
//
//        // Update the previous states of the buttons
//        prevDpadUp = gamepad1.dpad_up;
//        prevDpadDown = gamepad1.dpad_down;
//        prevDpadLeft = gamepad1.dpad_left;
//        prevDpadRight = gamepad1.dpad_right;
//        prevCross = gamepad1.cross; // Update the previous state of the cross button
//
//        long currentTime = System.currentTimeMillis();
//        if (!isLocked && currentTime - lastBlinkTime > 500) {
//            showNumber = !showNumber;
//            lastBlinkTime = currentTime;
//        }
//
//        CommandScheduler.getInstance().run();
//    }
//
//    @Override
//    public void start() {
//        time_since_start = new ElapsedTime();
//        movement8 = movement7.endTrajectory().fresh()
//                .setReversed(true)
//                .splineToSplineHeading(
//                        new Pose2d(31, lookupTable.getZoneInfo(submersibleFirstZone)[0], Math.toRadians(0)), Math.toRadians(180.00),
//                        null,
//                        new ProfileAccelConstraint(-85, 85)
//                )
//                .splineToSplineHeading(
//                        new Pose2d(27, lookupTable.getZoneInfo(submersibleFirstZone)[0], Math.toRadians(0)), Math.toRadians(180.00), //first sub grab
//                        null,
//                        new ProfileAccelConstraint(-85, 85)
//                );
//
//        movement82 = movement9.endTrajectory().fresh()
//                .setReversed(true)
//                .splineToSplineHeading(
//                        new Pose2d(31, lookupTable.getZoneInfo(submersibleSecondZone)[0], Math.toRadians(0)), Math.toRadians(180.00),
//                        null,
//                        new ProfileAccelConstraint(-85, 85)
//                )
//                .splineToSplineHeading(
//                        new Pose2d(27, lookupTable.getZoneInfo(submersibleSecondZone)[0], Math.toRadians(0)), Math.toRadians(180.00), //second sub grab
//                        null,
//                        new ProfileAccelConstraint(-85, 85)
//                );
//
//        movement8A = movement8.build();
//        movement8B = movement82.build();
//
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        //First Drop:
//                        new ParallelCommandGroup(
//                                new ActionCommand(movement1A, Collections.emptySet()),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(100),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                ),
//                                new SetIntakeDownCommand(robot)
//                        ),
//                        //First Intake:
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(0),
//                                        new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.38)
//                                ),
//                                new SequentialCommandGroup(
//                                        new BucketDropCommand(robot),
//                                        new OuttakeTransferReadyCommand(robot)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new ActionCommand(movement2A, Collections.emptySet())
//                                )
//                        ),
//                        new WaitCommand(250),
//                        new SlowIntakePeckerCommand(robot),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new RegularTransferCommand(robot),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(130),
//                                        new ActionCommand(movement3A, Collections.emptySet())
//                                )
//                        ),
//                        //Second Drop:
//                        //Second Intake:
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new ActionCommand(movement4A, Collections.emptySet())
//                                ),
//                                new SequentialCommandGroup(
//                                        new BucketDropCommand(robot),
//                                        new OuttakeTransferReadyCommand(robot)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(0),
//                                        new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.5)
//                                )
//                        ),
//                        new WaitCommand(150),
//                        new SlowIntakePeckerCommand(robot),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new RegularTransferCommand(robot),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(100),
//                                        new ActionCommand(movement5A, Collections.emptySet())
//                                )
//                        ),
//                        //Third Drop:
//                        //Fourth Intake:
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new ActionCommand(movement6A, Collections.emptySet())
//                                ),
//                                new SequentialCommandGroup(
//                                        new BucketDropCommand(robot),
//                                        new OuttakeTransferReadyCommand(robot)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(0),
//                                        new IntakeCommand(robot, 0.75, Globals.EXTENDO_MAX_EXTENSION * 0.38)
//                                )
//                        ),
//                        new WaitCommand(150),
//                        new SlowIntakePeckerCommand(robot),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new RetractedTransferCommand(robot),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(100),
//                                        new ActionCommand(movement7A, Collections.emptySet())
//                                )
//                        ),
//                        //Fourth Drop:
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new ActionCommand(movement8A, Collections.emptySet())
//                                ),
//                                new SequentialCommandGroup(
//                                        new BucketDropCommand(robot),
//                                        new OuttakeTransferReadyCommand(robot)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1800),
//                                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) Globals.EXTENDO_MAX_EXTENSION * lookupTable.getZoneInfo(submersibleFirstZone)[1])
//                                )
//                        ),
//                        //Vision stuff:
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {
//                                    isScanning = true;
//                                }),
//                                new WaitUntilCommand(() -> !isScanning),
//                                new ParallelCommandGroup(
//                                        new DeferredCommand(() ->
//                                                new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
//                                                robot.extendoSubsystem
//                                        ),
//                                        new DeferredCommand(() ->
//                                                new ActionCommand(
//                                                        robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
//                                                                .strafeToConstantHeading(new Vector2d(
//                                                                        robot.driveSubsystem.getPoseEstimate().position.x,
//                                                                        robot.driveSubsystem.getPoseEstimate().position.y + xTravel
//                                                                )).build()
//                                                        , Collections.emptySet())
//                                                , robot.driveSubsystem)
//                                ),
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {
//                                    isScanning = true;
//                                }),
//                                new WaitUntilCommand(() -> !isScanning),
//                                new ParallelCommandGroup(
//                                        new DeferredCommand(() ->
//                                                new IntakeCommand(robot, lastEstimate, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
//                                                robot.extendoSubsystem
//                                        ),
//                                        new DeferredCommand(() ->
//                                                new ActionCommand(
//                                                        robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
//                                                                .strafeToConstantHeading(new Vector2d(
//                                                                        robot.driveSubsystem.getPoseEstimate().position.x,
//                                                                        robot.driveSubsystem.getPoseEstimate().position.y + xTravel
//                                                                )).build()
//                                                        , Collections.emptySet())
//                                                , robot.driveSubsystem)
//                                ),
//                                new IntakePeckerCommand(robot)
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new RegularTransferCommand(robot),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement9A, Collections.emptySet())
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new BucketDropCommand(robot),
//                                        new OuttakeTransferReadyCommand(robot)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new ActionCommand(movement8B, Collections.emptySet())
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1800),
//                                        new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) Globals.EXTENDO_MAX_EXTENSION * lookupTable.getZoneInfo(submersibleSecondZone)[1])
//                                )
//                        ),
//                        //Vision stuff:
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {
//                                    isScanning = true;
//                                }),
//                                new WaitUntilCommand(() -> !isScanning),
//                                new ParallelCommandGroup(
//                                        new DeferredCommand(() ->
//                                                new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
//                                                robot.extendoSubsystem
//                                        ),
//                                        new DeferredCommand(() ->
//                                                new ActionCommand(
//                                                        robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
//                                                                .strafeToConstantHeading(new Vector2d(
//                                                                        robot.driveSubsystem.getPoseEstimate().position.x,
//                                                                        robot.driveSubsystem.getPoseEstimate().position.y + xTravel
//                                                                )).build()
//                                                        , Collections.emptySet())
//                                                , robot.driveSubsystem)
//                                ),
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {
//                                    isScanning = true;
//                                }),
//                                new WaitUntilCommand(() -> !isScanning),
//                                new ParallelCommandGroup(
//                                        new DeferredCommand(() ->
//                                                new IntakeCommand(robot, lastEstimate, (double) robot.extendoMotor.getCurrentPosition() - (Globals.EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES * yTravel)),
//                                                robot.extendoSubsystem
//                                        ),
//                                        new DeferredCommand(() ->
//                                                new ActionCommand(
//                                                        robot.driveSubsystem.trajectoryActionBuilder(robot.driveSubsystem.getPoseEstimate())
//                                                                .strafeToConstantHeading(new Vector2d(
//                                                                        robot.driveSubsystem.getPoseEstimate().position.x,
//                                                                        robot.driveSubsystem.getPoseEstimate().position.y + xTravel
//                                                                )).build()
//                                                        , Collections.emptySet())
//                                                , robot.driveSubsystem)
//                                ),
//                                new IntakePeckerCommand(robot)
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new RegularTransferCommand(robot),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ActionCommand(movement9B, Collections.emptySet())
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(300),
//                                        new ActionCommand(movement10A, Collections.emptySet())
//                                ),
//                                new SequentialCommandGroup(
//                                        new BucketDropCommand(robot),
//                                        new OuttakeTransferReadyCommand(robot)
//                                )
//                        )
//                )
//        );
//    }
//
//    @Override
//    public void loop() {
//        CommandScheduler.getInstance().run();
//        robot.driveSubsystem.updatePoseEstimate();
//        robot.depositSubsystem.outtakeSlidesLoop(0.0002);
//        robot.extendoSubsystem.extendoSlidesLoop(0.013,0,0.00025,0);
//
//        if (isScanning) {
//            double greenAngle = sampleDetection.getAngleOfGreenSample();
//            Point greenPoint = sampleDetection.getGreenSampleCoordinates();
//
//            if (!Double.isNaN(greenAngle)) {
//                xTravel = greenPoint.x;
//                yTravel = greenPoint.y;
//                double angle = (greenAngle % 180) / 180;
//                lastEstimate = angle;
//                Log.i("Successful Scan (Blue 0+6):", "xTravel: " + xTravel + "yTravel: " + yTravel);
//                isScanning = false;
//            } else {
//                angle = lastEstimate;
//            }
//        }
//
//        loop = time;
//        telemetry.update();
//        robot.clearCache();
//    }
//
//    @Override
//    public void stop() {
//        telemetry.addLine("Ended OpMode.");
//        telemetry.update();
//        CommandScheduler.getInstance().reset();
//    }
//}