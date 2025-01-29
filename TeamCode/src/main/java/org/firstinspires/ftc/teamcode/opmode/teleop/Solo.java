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
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.NoClawScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleopspecific.CustomBucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleopspecific.CustomOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.RegularTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.RetractedTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.IntakeSliderResetterCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class Solo extends CommandOpMode {
    public static final double[] intakeRotationPositions = {0, 0.25, 0.55, 0.75, 1};
    Gamepad soloController;
    GamepadEx soloButtonController;
    boolean extendoBoolean = true;
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean driverControlUnlocked;
    private boolean isCloseAndTransfer = true; // Track toggle state
    private int currentIndex = 2; //for rotation

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, false);
        soloController = gamepad1;

        robot.leftLift.setCurrentAlert(9.0, CurrentUnit.AMPS);

        soloButtonController = new GamepadEx(gamepad1);

        driverControlUnlocked = true;
        depositManualControl = true;
        isCloseAndTransfer = true; // Track toggle state

        soloButtonController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> depositManualControl = false),
                        new SpecimenIntakeCommand(robot)
                )
        );

        soloButtonController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new SpecimenGrabAndTransferAndLiftCommand(robot),
                        new InstantCommand(() -> depositManualControl = false)
                )
        );

        soloButtonController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new UninterruptableCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> depositManualControl = false),
                                new SpecimenClipCommand(robot),
                                new WaitCommand(300),
                                new SpecimenReadyCommand(robot)
                        )
                )
        );

        soloButtonController.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    currentIndex = (currentIndex + 2) % intakeRotationPositions.length;
                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
                })
        );

        soloButtonController.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    currentIndex = (currentIndex - 1 + intakeRotationPositions.length) % intakeRotationPositions.length; // Wrap to the end
                    robot.intakeRotation.setPosition(intakeRotationPositions[currentIndex]);
                })
        );

        soloButtonController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            if (isCloseAndTransfer) {
                new IntakePeckerCommand(robot).schedule();
            } else {
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()).schedule();
            }
            isCloseAndTransfer = !isCloseAndTransfer;
        });

        soloButtonController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
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
        telemetry.addData("Slides Current: ", robot.leftLift.getCurrent(CurrentUnit.AMPS));


        if (robot.leftLift.isOverCurrent() && robot.leftLift.getPower() < 0) {
            robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (driverControlUnlocked) {
            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.48 * Math.tan(1.12 * soloController.left_stick_y),
                            0.48 * Math.tan(1.12 * soloController.left_stick_x)
                    ),
                    -soloController.right_stick_x
            ));
        }

        //Extendo Slides Stuff:
        if (extendoBoolean) {
            robot.extendoSubsystem.extendoSlidesLoop();
        }

        telemetry.update();

        if (soloController.cross) {
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

        if (soloController.circle) {
            extendoBoolean = true;
            schedule(
                    new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        //Deposit Slides Stuff:
        if (!depositManualControl) {
            robot.depositSubsystem.outtakeSlidesLoop();
        }

        if (soloController.dpad_right) {
            extendoBoolean = false;
            schedule(
                    new ParallelCommandGroup(
                            new CustomOuttakeCommand(robot),
                            new InstantCommand(() -> {
                                extendoBoolean = false;
                            }),
                            new IntakeSliderResetterCommand(robot.extendoSubsystem)
                    )
            );
        }
        
        if (soloController.dpad_left) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_RETRACT_POS)
            );
        }

        if (soloController.left_trigger > 0.5) {
            robot.leftHang.setPower(1);
            robot.rightHang.setPower(1);
        }

        if (soloController.left_trigger < 0.5) {
            robot.leftHang.setPower(0);
            robot.rightHang.setPower(0);
        }

        if (soloController.right_trigger > 0.5) {
            robot.leftHang.setPower(-1);
            robot.rightHang.setPower(-1);
        }

        //Overrides:
        if (soloController.ps) {
            soloController.rumble(1000);
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

        telemetry.addData("Extendo State: ", Globals.extendoState);
        telemetry.addData("Outtake State: ", Globals.outtakeState);
        telemetry.addData("Intake Rotation State: ", Globals.intakeRotationState);
        telemetry.addData("Intake Coaxial State: ", Globals.intakeCoaxialState);
        telemetry.addData("Intake Claw State: ", Globals.intakeClawState);
        telemetry.addData("FourBar State: ", Globals.fourBarState);
        telemetry.addData("Outtake Arm State: ", Globals.outtakeArmState);
        telemetry.addData("Outtake Claw State: ", Globals.outtakeClawState);
    }
}
