package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.UninterruptableCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SecondarySpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop.CLCloseAndTransfer;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop.CLRetractedCloseAndTransfer;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class TeleOpR extends CommandOpMode {
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean driverControlUnlocked;
    public static final double[] intakeRotationPositions = {0.83, 0.6925, 0.555, 0.4175, 0.28};
    Gamepad ahnafController, swethaController;
    GamepadEx ahnafLigmaController, swethaLigmaController;

    Globals.OuttakeClawState outtakeClawState;
    Globals.OuttakeArmState outtakeArmState;
    Globals.FourBarState fourBarState;
    Globals.IntakeClawState intakeClawState;
    Globals.IntakeCoaxialState intakeCoaxialState;
    Globals.IntakeRotationState intakeRotationState;
    Globals.ExtendoState extendoState;
    Globals.OuttakeState outtakeState;

    private boolean isCloseAndTransfer = true; // Track toggle state

    double robotPitch;
    double antiTipPower;
    private int currentIndex = 0; //for rotation

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, false);
        ahnafController = gamepad1;
        swethaController = gamepad2;

        robot.leftLift.setCurrentAlert(9.0, CurrentUnit.AMPS);

        ahnafLigmaController = new GamepadEx(gamepad1);
        swethaLigmaController = new GamepadEx(gamepad2);

        driverControlUnlocked = true;
        depositManualControl = true;
        isCloseAndTransfer = true; // Track toggle state

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> depositManualControl = false),
                        new SpecimenIntakeCommand(robot)
                )
        );

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new SpecimenGrabAndTransferAndLiftCommand(robot),
                        new InstantCommand(() -> depositManualControl = false)
                )
        );

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptableCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> depositManualControl = false),
//                                new SpecimenClipCommand(robot),
                                new SecondarySpecimenClipCommand(robot), //NEW - TEST //TODO: TEST TEST TEST
                                new WaitCommand(300),
                                new SpecimenReadyCommand(robot)
                        )
                )
        );

        swethaLigmaController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    currentIndex = (currentIndex + 1) % intakeRotationPositions.length;
                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
                })
        );

        swethaLigmaController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    currentIndex = (currentIndex - 1 + intakeRotationPositions.length) % intakeRotationPositions.length; // Wrap to the end
                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
                })
        );

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            currentIndex = 0;
            if (isCloseAndTransfer) {
                new IntakePeckerCommand(robot).schedule();
            } else {
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()).schedule();
            }
            isCloseAndTransfer = !isCloseAndTransfer;
        });

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new BucketDropCommand(robot)
        );
    }

    @Override
    public void run() {

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        telemetry.addData("Slides Current: ", robot.leftLift.getCurrent(CurrentUnit.AMPS));


        if (robot.leftLift.isOverCurrent() && robot.leftLift.getPower() < 0) {
            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Ahnaf's Controls:

        if (driverControlUnlocked) {
            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.48 * Math.tan(1.12 * ahnafController.left_stick_y),
                            0.48 * Math.tan(1.12 * ahnafController.left_stick_x) //48 is more fire than 50 lmao. graph it on desmos!
                    ),
                    -ahnafController.right_stick_x
            ));
        }


        //Swetha's Controls:
        //Extendo Slides Stuff:

        robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);

        telemetry.addData("Extendo Position: ", robot.extendoMotor.getCurrentPosition());
//        telemetry.addData("Extendo State: ", extendoState.toString());
//        telemetry.addData("Outtake State: ", outtakeState.toString());
//        telemetry.addData("Intake Rotation State: ", intakeRotationState.toString());
//        telemetry.addData("Intake Coaxial State: ", intakeCoaxialState.toString());
//        telemetry.addData("Intake Claw State: ", intakeClawState.toString());
//        telemetry.addData("FourBar State: ", fourBarState.toString());
//        telemetry.addData("Outtake Arm State: ", outtakeArmState.toString());
//        telemetry.addData("Outtake Claw State: ", outtakeClawState.toString());
        telemetry.update();

        if (ahnafController.cross) {
            if (robot.extendoMotor.getCurrentPosition() > 800) {
                schedule(
                        new UninterruptableCommand(new CLCloseAndTransfer(robot)),
                        new InstantCommand(() -> isCloseAndTransfer = true)
                );
            }
            if (robot.extendoMotor.getCurrentPosition() < 800) {
                schedule(
                        new UninterruptableCommand(new CLRetractedCloseAndTransfer(robot)),
                        new InstantCommand(() -> isCloseAndTransfer = true)
                );
            }
        }

        if (swethaController.circle) {
            schedule(
                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        if (swethaController.square) {
            schedule(
                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, ((double) Globals.EXTENDO_MAX_EXTENSION / 2))
            );
        }

        if (swethaController.triangle) {
            schedule(
                    new ScanningCommand(robot, 0.5, ((double) Globals.EXTENDO_MAX_EXTENSION / 4))
            );
        }

        if (swethaController.cross) {
            schedule(
                    new ScanningCommand(robot, 0.5, Globals.EXTENDO_MAX_RETRACTION)
            );
        }

        //Deposit Slides Stuff:
        if (!depositManualControl) {
            robot.depositSubsystem.outtakeSlidesLoop(Globals.LIFT_P_SLOW);
        }
        if (depositManualControl) {
            robot.depositSubsystem.depositManualControlLoop(-swethaController.right_stick_y);
        }

        if (swethaController.right_stick_y > 0) {
            depositManualControl = true;
        }
        if (swethaController.dpad_left || swethaController.dpad_right || swethaController.dpad_up || swethaController.dpad_down) {
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            depositManualControl = false;
        }

        if (swethaController.dpad_up) {
            if (robot.extendoMotor.getCurrentPosition() < 100) {
                robot.extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
            );
        }

        if (swethaController.dpad_right) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_MID_POS)
            );
        }

        if (swethaController.dpad_left) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_PARK_POS)
            );
        }

        if (swethaController.dpad_down) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_RETRACT_POS)
            );
        }

        if (swethaController.left_trigger > 0.5 || swethaController.right_trigger > 0.5) {
            schedule(
                    new OuttakeTransferReadyCommand(robot)
            );
        }


        //Overrides:
        if (ahnafController.ps) {
            ahnafController.rumble(1000);
            swethaController.rumble(1000);
            schedule(
                    new AllSystemInitializeCommand(robot)
            );
        }
    }
}
