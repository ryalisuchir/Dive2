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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class OldTeleOpX extends CommandOpMode {
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean extendoManualControl;
    private boolean driverControlUnlocked;

    Gamepad ahnafController, swethaController;
    GamepadEx ahnafLigmaController;
    Gamepad ahnafPreviousGamepad = new Gamepad();

    double robotPitch;
    double antiTipPower;

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
        ahnafController = gamepad1;
        swethaController = gamepad2;

        ahnafLigmaController = new GamepadEx(gamepad1);

        driverControlUnlocked = true;
        depositManualControl = true;
        extendoManualControl = true;
    }

    @Override
    public void run() {

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.currentLoop();

        //Ahnaf's Controls:

        if (driverControlUnlocked) {
            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
                            0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
                    ),
                    -0.5 * Math.tan(1.12 * ahnafController.right_stick_x)
            ));
        }

        if (ahnafController.cross) {
            schedule(
                    new CloseAndTransferCommand(robot)
            );
        }

        if (ahnafController.circle) {
            schedule(
                    new IntakeCommand(robot, 0.5, robot.extendoMotor.getCurrentPosition())
            );
        }

        if (ahnafController.square) {
            schedule(
                    new ScanningCommand(robot, 0.5, (double) Globals.EXTENDO_MAX_EXTENSION / 2)
            );
        }

        if (ahnafController.triangle) {
            schedule(
                    new BucketDropCommand(robot)
            );
        }


        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            double currentPosition = robot.intakeRotation.getPosition();
            double newPosition = Globals.INTAKE_ROTATION_ZERO +
                    ((currentPosition - Globals.INTAKE_ROTATION_ZERO + Globals.INTAKE_ROTATION_INCREMENT) %
                            (5 * Globals.INTAKE_ROTATION_INCREMENT));
            robot.intakeRotation.setPosition(newPosition);
        });

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            double currentPosition = robot.intakeRotation.getPosition();
            double newPosition = Globals.INTAKE_ROTATION_ZERO +
                    ((currentPosition - Globals.INTAKE_ROTATION_ZERO - Globals.INTAKE_ROTATION_INCREMENT +
                            5 * Globals.INTAKE_ROTATION_INCREMENT) %
                            (5 * Globals.INTAKE_ROTATION_INCREMENT));
            robot.intakeRotation.setPosition(newPosition);
        });

        //Swetha's Controls:
        //Extendo Slides Stuff:

        if (!extendoManualControl) {
            robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);
        }

        if (extendoManualControl) {
            robot.extendoSubsystem.extendoManualControlLoop(swethaController.left_stick_x);
        }

        if (swethaController.left_stick_x > 0) {
            extendoManualControl = true;
        }

        if (swethaController.square || swethaController.circle || swethaController.dpad_right || swethaController.dpad_left) {
            extendoManualControl = false;
        }

        if (swethaController.circle) {
            schedule(
                    new ScanningCommand(robot, 0.5, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        if (swethaController.square) {
            schedule(
                    new ScanningCommand(robot, 0.5, ((double) Globals.EXTENDO_MAX_EXTENSION / 2))
            );
        }

        if (swethaController.dpad_right) {
            schedule(
                    new ScanningCommand(robot, 0.5, ((double) Globals.EXTENDO_MAX_EXTENSION / 4))
            );
        }

        if (swethaController.dpad_left) {
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
        if (swethaController.triangle || swethaController.cross || swethaController.dpad_up || swethaController.dpad_down) {
            depositManualControl = false;
        }

        if (swethaController.triangle) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
            );
        }

        if (swethaController.cross) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_MID_POS)
            );
        }

        if (swethaController.dpad_up) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
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

        //Anti-Tip:
        robotPitch = robot.pinpointDrive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);

        if (robotPitch > 2) {
            ahnafController.rumble(500);
            swethaController.rumble(500);
        }

        if (
                robotPitch > 5 ||
                        robotPitch < -5
        ) {

            if (robotPitch > 5) {
                antiTipPower = -1;
            }
            if (robotPitch < -5) {
                antiTipPower = 1;
            }
            ahnafController.rumble(500);
            swethaController.rumble(500);
            driverControlUnlocked = false;
            schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new AllSystemInitializeCommand(robot),
                                    new InstantCommand(() -> robot.driveSubsystem.setDrivePowers(0, antiTipPower, 0))
                            ),
                            new WaitCommand(500),
                            new InstantCommand(() -> robot.driveSubsystem.setDrivePowers(0, 0, 0))
                    )
            );
            driverControlUnlocked = true;
            ahnafController.rumble(1000);
            swethaController.rumble(1000);
        }

    }
}
