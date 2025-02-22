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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.custom.CustomBucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.custom.CustomHighBucketCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.custom.CustomLowBucketCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.RetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen.SpecimenReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.RegularTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.RetractedTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.IntakeSliderResetterCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.UninterruptableCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class Duo extends CommandOpMode {
    public static final double[] intakeRotationPositions = {0, 0.25, 0.55, 0.75, 1};
    Gamepad ahnafController, swethaController;
    GamepadEx ahnafButtonController, swethaButtonController;
    boolean extendoBoolean = true;
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean driverControlUnlocked;
    private boolean isCloseAndTransfer = true; // Track toggle state

    private int currentIndex = 2; //for rotation

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, false);
        ahnafController = gamepad1;
        swethaController = gamepad2;

        ahnafButtonController = new GamepadEx(gamepad1);
        swethaButtonController = new GamepadEx(gamepad2);

        driverControlUnlocked = true;
        depositManualControl = true;
        isCloseAndTransfer = true; // Track toggle state

        ahnafButtonController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> depositManualControl = false),
                        new SpecimenIntakeCommand(robot)
                )
        );

        ahnafButtonController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new SpecimenGrabAndTransferAndLiftCommand(robot),
                        new InstantCommand(() -> depositManualControl = false)
                )
        );

        ahnafButtonController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptableCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> depositManualControl = false),
                                new SpecimenClipCommand(robot),
                                new WaitCommand(300),
                                new SpecimenReadyCommand(robot)
                        )
                )
        );

        swethaButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    currentIndex = (currentIndex + 1) % intakeRotationPositions.length;
                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
                })
        );

        swethaButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    currentIndex = (currentIndex - 1 + intakeRotationPositions.length) % intakeRotationPositions.length; // Wrap to the end
                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
                })
        );

        ahnafButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            if (isCloseAndTransfer) {
                new IntakePeckerCommand(robot).schedule();
            } else {
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()).schedule();
            }
            isCloseAndTransfer = !isCloseAndTransfer;
        });

        ahnafButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new CustomBucketDropCommand(robot),
                        new InstantCommand(() -> {
                            extendoBoolean = true;
                        })
                )
        );

    }

    @Override
    public void run() {

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();

        //Ahnaf's Controls:

        if (driverControlUnlocked) {
            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.48 * Math.tan(1.12 * ahnafController.left_stick_y),
                            0.48 * Math.tan(1.12 * ahnafController.left_stick_x)
                    ),
                    -ahnafController.right_stick_x
            ));
        }

        //Swetha's Controls:
        //Extendo Slides Stuff:
        if (extendoBoolean) {
            robot.extendoSubsystem.extendoSlidesLoop(0.015, 0, 0, 0);
        }

        if (swethaController.left_stick_x > 0.1) {
            extendoBoolean = false;
            robot.extendoMotor.setPower(-1);
        } else {
            extendoBoolean = true;
        }

        telemetry.update();

        if (ahnafController.cross) {
            currentIndex = 2;
            if (robot.extendoMotor.getCurrentPosition() > (Globals.EXTENDO_MAX_EXTENSION / 2) + 50) {
                schedule(
                        new UninterruptableCommand(new RegularTransferCommand(robot)),
                        new InstantCommand(() -> isCloseAndTransfer = true)
                );
            }
            if (robot.extendoMotor.getCurrentPosition() < (Globals.EXTENDO_MAX_EXTENSION / 2) + 50) {
                schedule(
                        new UninterruptableCommand(new RetractedTransferCommand(robot)),
                        new InstantCommand(() -> isCloseAndTransfer = true)
                );
            }
        }

        if (swethaController.circle) {
            extendoBoolean = true;
            schedule(
                    new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        if (swethaController.square) {
            extendoBoolean = true;
            schedule(
                    new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, ((double) Globals.EXTENDO_MAX_EXTENSION / 2))
            );
        }

        if (swethaController.triangle) {
            extendoBoolean = true;
            schedule(
                    new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, ((double) Globals.EXTENDO_MAX_EXTENSION / 4))
            );
        }
        if (ahnafController.triangle) {
            currentIndex = 2;
            extendoBoolean = true;
            schedule(
                    new RetractCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION)
            );
        }

        if (swethaController.cross) {
            currentIndex = 2;
            extendoBoolean = true;
            schedule(
                    new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION)
            );
        }

        //Deposit Slides Stuff:
        if (!depositManualControl) {
            robot.depositSubsystem.outtakeSlidesLoop();
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
            extendoBoolean = false;
            schedule(
                    new ParallelCommandGroup(
                            new CustomHighBucketCommand(robot),
                            new InstantCommand(() -> {
                                extendoBoolean = false;
                            }),
                            new IntakeSliderResetterCommand(robot.extendoSubsystem)
                    )
            );
        }

        if (swethaController.dpad_right) {
            schedule(
                    new CustomLowBucketCommand(robot)
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

        if (swethaController.left_trigger > 0.5) {
            robot.leftHang.setPower(1);
            robot.rightHang.setPower(1);
        }

        if (swethaController.left_trigger < 0.5) {
            robot.leftHang.setPower(0);
            robot.rightHang.setPower(0);
        }

        if (swethaController.right_trigger > 0.5) {
            robot.leftHang.setPower(-1);
            robot.rightHang.setPower(-1);
        }

        //Overrides:
        if (ahnafController.ps) {
            ahnafController.rumble(1000);
            swethaController.rumble(1000);
            schedule(
                    new AllSystemInitializeCommand(robot)
            );
            robot.extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
}
