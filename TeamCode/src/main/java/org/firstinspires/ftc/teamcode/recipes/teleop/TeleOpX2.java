//package org.firstinspires.ftc.teamcode.recipes.teleop;
//
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.slow.SlowCloseAndTransferCommand;
//import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
//import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;
//
//@TeleOp
//public class TeleOpX2 extends CommandOpMode {
//    //Setting variables:
//    private RobotHardware robot;
//    private boolean depositManualControl;
//    private boolean extendoManualControl;
//    private boolean driverControlUnlocked;
//
//    Gamepad ahnafController, swethaController;
//    GamepadEx ahnafButtonController, swethaButtonController;
//    Gamepad ahnafPreviousGamepad = new Gamepad();
//
//    private int currentIndex = 0; //for rotation
//    public static final double[] intakeRotationPositions = { 0.83, 0.7383, 0.64667, 0.555, 0.46333, 0.37167, 0.28 };
//
//    private double depositP, extendoP;
//    private double stickThreshold;
//
//    @Override
//    public void initialize() {
//        //RobotHardware:
//        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, false);
//
//
//        if (robot.rightLift.isOverCurrent() && robot.rightLift.getPower() < 0) {
//            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        //Gamepads:
//        ahnafController = gamepad1;
//        swethaController = gamepad2;
//        ahnafButtonController = new GamepadEx(gamepad1);
//        swethaButtonController = new GamepadEx(gamepad2);
//
//        driverControlUnlocked = true;
//        depositManualControl = true;
//        extendoManualControl = true;
//
//        stickThreshold = 0.05;
//        depositP = 0.001;
//        extendoP = 0.001;
//
//        //Ahnaf Button Controls:
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(() -> new SlowCloseAndTransferCommand(robot).schedule(false))
//        );
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, robot.extendoMotor.getCurrentPosition())
//        );
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new BucketDropCommand(robot)
//        );
//
//        //Swetha Button Controls:
//
//        //Intake Claw Rotation:
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    currentIndex = (currentIndex + 1) % intakeRotationPositions.length;
//                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
//                })
//        );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    currentIndex = (currentIndex - 1 + intakeRotationPositions.length) % intakeRotationPositions.length; // Wrap to the end
//                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
//                })
//        );
//
//        //Slide PID Switches:
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new ParallelCommandGroup(
//                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
//                        new InstantCommand(() -> {
//            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            depositManualControl = false;
//        })
//                )
//        );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(() -> {
//                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    depositManualControl = false;
//                })
//        );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> {
//                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    depositManualControl = false;
//                })
//        );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new InstantCommand(() -> {
//                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    depositManualControl = false;
//                })
//        );
//
//
//    }
//
//    @Override
//    public void run() {
//        //Loop:
//        CommandScheduler.getInstance().run();
//        robot.driveSubsystem.updatePoseEstimate();
//        robot.clearCache();
//        telemetry.addData("Linear Slides Position: ", robot.rightLift.getCurrentPosition());
//        telemetry.addData("Extendo Slides Position: ", robot.extendoMotor.getCurrentPosition());
//        telemetry.update();
//
//        //Extendo Loops:
//        if (!extendoManualControl) {
//            robot.extendoSubsystem.extendoSlidesLoop(extendoP);
//        }
//        if (extendoManualControl) {
//            robot.extendoSubsystem.extendoManualControlLoop(swethaController.left_stick_x);
//        }
//        if (swethaController.left_stick_x > stickThreshold) {
//            extendoManualControl = true;
//        }
//
//        //Deposit Slide Loops:
//        if (!depositManualControl) {
//            robot.depositSubsystem.outtakeSlidesLoop(depositP);
//        }
//        if (depositManualControl) {
//            robot.depositSubsystem.depositManualControlLoop(-swethaController.right_stick_y);
//        }
//        if (swethaController.right_stick_y > stickThreshold) {
//            depositManualControl = true;
//        }
//
//        //Driving:
//        if (driverControlUnlocked) {
//            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
//                            0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
//                    ),
//                    -ahnafController.right_stick_x
//            ));
//        }
//
//        //Failsafes:
//        if (ahnafController.ps) {
//            ahnafController.rumble(1000);
//            swethaController.rumble(1000);
//            schedule(
//                    new InstantCommand(() -> new AllSystemInitializeCommand(robot).schedule(false))
//            );
//        }
//
//    }
//}
