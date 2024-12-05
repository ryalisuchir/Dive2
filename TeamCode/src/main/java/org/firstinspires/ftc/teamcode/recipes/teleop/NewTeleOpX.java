//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import static org.firstinspires.ftc.teamcode.common.hardware.teleop.TeleOpGlobals.intakeRotationPositions;
//
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.SpecimenIntakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.SpecimenClipCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.TAllSystemRestCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake.TPeckerCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake.TIntakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.outtake.TBucketDropCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.ground.TTransferCommand;
//import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
//import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.hardware.teleop.TeleOpGlobals;
//
//@TeleOp
//public class NewTeleOpX extends CommandOpMode {
//    private RobotHardware robot;
//    private boolean depositManualControl;
//    private boolean extendoManualControl;
//    private boolean driverControlUnlocked;
//
//    Gamepad ahnafController, swethaController;
//    GamepadEx ahnafButtonController, swethaButtonController;
//
//    double robotPitch;
//    double antiTipPower;
//
//    private int currentIndex = 0; //for rotation
//
//    @Override
//    public void initialize() {
//        //Do not remove:
//        CommandScheduler.getInstance().reset();
//        robot = new RobotHardware(hardwareMap, TeleOpGlobals.DEFAULT_START_POSE, false); //Registers subsystems too
//
//        //Setting controllers:
//        ahnafController = gamepad1;
//        swethaController = gamepad2;
//        ahnafButtonController = new GamepadEx(gamepad1);
//        swethaButtonController = new GamepadEx(gamepad2);
//
//        //All manual controls:
//        driverControlUnlocked = true;
//        depositManualControl = true;
//        extendoManualControl = true;
//
//        //Ahnaf Controls:
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    currentIndex = (currentIndex + 1) % intakeRotationPositions.length;
//                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
//                })
//        );
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    currentIndex = (currentIndex - 1 + intakeRotationPositions.length) % intakeRotationPositions.length; // Wrap to the end
//                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
//                })
//        );
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new TPeckerCommand(
//                        robot,
//                        TeleOpGlobals.INTAKE_FOURBAR_LOW //fourbar position for the drop as it grabs
//                ));
//
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.A).whenPressed( //cross
//                new TTransferCommand(
//                        robot,
//                        TeleOpGlobals.INTAKE_FOURBAR_TRANSFER, //fourbar position when it transfers
//                        TeleOpGlobals.INTAKE_COAXIAL_TRANSFER //coaxial position when it transfers
//                ));
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.X).whenPressed( //square
//                new TIntakeCommand(
//                        robot,
//                        TeleOpGlobals.INTAKE_CLAW_OPEN, //claw opening position
//                        TeleOpGlobals.INTAKE_FOURBAR_SCANNING, //fourbar higher than intake position to go over submersible
//                        TeleOpGlobals.INTAKE_COAXIAL_INTAKE, //coaxial position/zombie axle position
//                        TeleOpGlobals.INTAKE_ROTATION_REST, //180 with robot
//                        0 //no extendo
//                ));
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.B).whenPressed( //circle
//                new TPeckerCommand(
//                        robot,
//                        TeleOpGlobals.INTAKE_FOURBAR_INTAKE //how low fourbar should go for intake
//                ));
//
//        ahnafButtonController.getGamepadButton(GamepadKeys.Button.Y).whenPressed( //triangle
//           new TBucketDropCommand(
//                   robot,
//                   TeleOpGlobals.OUTTAKE_ARM_DUNK, //how low outtake arm should drop into bucket
//                   TeleOpGlobals.OUTTAKE_CLAW_OPEN //how much outtake claw opens
//           ));
//
//        //Swetha Controls:
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.B).whenPressed( //Circle
//                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, TeleOpGlobals.EXTENDO_MAX_EXTENSION)
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.X).whenPressed( //Square
//                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, ((double) TeleOpGlobals.EXTENDO_MAX_EXTENSION / 2))
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed( //Dpad_Right
//                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, ((double) TeleOpGlobals.EXTENDO_MAX_EXTENSION / 4))
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed( //Dpad_Left
//                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, TeleOpGlobals.EXTENDO_MAX_RETRACTION)
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.Y).whenPressed( //Triangle
//                    new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//            );
//
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.A).whenPressed( //Cross
//                    new OuttakeCommand(robot, Globals.LIFT_MID_POS)
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed( //Dpad_Up
//                    new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed( //Dpad_Down
//                    new OuttakeCommand(robot, Globals.LIFT_RETRACT_POS)
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed( //Left Bumper
//                    new OuttakeTransferReadyCommand(robot)
//            );
//
//        swethaButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed( //Right Bumper
//                new SpecimenIntakeCommand(robot)
//        );
//
//    }
//
//    @Override
//    public void run() {
//
//        //Loop:
//        CommandScheduler.getInstance().run();
//        robot.driveSubsystem.updatePoseEstimate();
//        robot.extendoSubsystem.currentLoop();
//
//
//        if (driverControlUnlocked) {
//            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
//                            0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
//                    ),
//                    -ahnafController.right_stick_x //Making turns normal again
//            ));
//        }
//
//        //Swetha Controls:
//        if (swethaController.right_trigger > 0 || swethaController.left_trigger > 0) {
//            new SpecimenClipCommand(robot);
//        }
//
//        //Slide Controls:
//
//        //Extendo Slides:
//        if (!extendoManualControl) {
//            robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);
//        }
//
//        if (extendoManualControl) {
//            robot.extendoSubsystem.extendoManualControlLoop(swethaController.left_stick_x);
//        }
//
//        if (swethaController.left_stick_x > 0) {
//            extendoManualControl = true;
//        }
//
//        if (swethaController.square || swethaController.circle || swethaController.dpad_right || swethaController.dpad_left) {
//            extendoManualControl = false;
//        }
//
//        //Deposit/Lift Slides:
//        if (!depositManualControl) {
//            robot.depositSubsystem.outtakeSlidesLoop(Globals.LIFT_P_SLOW);
//        }
//        if (depositManualControl) {
//            robot.depositSubsystem.depositManualControlLoop(-swethaController.right_stick_y);
//        }
//
//        if (swethaController.right_stick_y > 0) {
//            depositManualControl = true;
//        }
//        if (swethaController.triangle || swethaController.cross || swethaController.dpad_up || swethaController.dpad_down) {
//            depositManualControl = false;
//        }
//
//        //Failsafes:
//        if (ahnafController.ps) {
//            ahnafController.rumble(1000);
//            swethaController.rumble(1000);
//            schedule(
//                    new TAllSystemRestCommand(
//                            robot,
//                            TeleOpGlobals.INTAKE_CLAW_OPEN, //intakeClawInput
//                            TeleOpGlobals.INTAKE_FOURBAR_RESTING, //intakeFourBarInput
//                            TeleOpGlobals.INTAKE_COAXIAL_RESTING, //intakeCoaxialInput
//                            TeleOpGlobals.INTAKE_ROTATION_REST, //intakeRotationInput
//                            TeleOpGlobals.EXTENDO_MAX_RETRACTION, //extendoPositionInput
//                            TeleOpGlobals.LIFT_RETRACT_POS, //depositPositionInput
//                            TeleOpGlobals.OUTTAKE_CLAW_OPEN, //outtakeClawInput
//                            TeleOpGlobals.OUTTAKE_ARM_TRANSFER //outtakeArmInput
//                    )
//            );
//        }
//
//        robotPitch = robot.pinpointDrive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
//
//        if (robotPitch > 4) {
//            ahnafController.rumble(500);
//            swethaController.rumble(500);
//        }
//
//    }
//}
