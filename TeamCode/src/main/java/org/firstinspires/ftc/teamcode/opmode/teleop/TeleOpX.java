package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class TeleOpX extends CommandOpMode {
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean extendoManualControl;

    boolean previousLeftBumper, currentLeftBumper; //For Rising-Edge Detector
    boolean previousRightBumper, currentRightBumper; //For Rising-Edge Detector

    Gamepad ahnafController, swethaController;
    Gamepad ahnafPreviousGamepad = new Gamepad();

    final double RESET_POSITION = 1.0;
    final double ZERO_POSITION = 0.0;
    final double POSITION_INCREMENT = 0.25;
    final double POSITION_THRESHOLD = 0.01;

    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
        ahnafController = gamepad1;
        swethaController = gamepad2;
    }

    @Override
    public void run() {

        //Rising-Edge Detector:
        ahnafPreviousGamepad.copy(ahnafController);
        ahnafController.copy(ahnafPreviousGamepad);

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();

        //Ahnaf's Controls:
        robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
                        -0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
                ),
                -0.5 * Math.tan(1.12 * ahnafController.right_stick_x)
        ));

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
                    new ScanningCommand(robot, 0.5, robot.extendoMotor.getCurrentPosition())
            );
        }

        if (ahnafController.triangle) {
            schedule(
                    new BucketDropCommand(robot)
            );
        }

        if (ahnafController.left_bumper && !ahnafPreviousGamepad.left_bumper) {
            if (Math.abs(robot.intakeRotation.getPosition() - RESET_POSITION) < POSITION_THRESHOLD) {
                double newPosition = Math.max(ZERO_POSITION, robot.intakeRotation.getPosition() - POSITION_INCREMENT);
                robot.intakeRotation.setPosition(newPosition);
            } else {
                robot.intakeRotation.setPosition(RESET_POSITION);
            }
        }

        if (ahnafController.right_bumper && !ahnafPreviousGamepad.right_bumper) {
            if (Math.abs(robot.intakeRotation.getPosition() - RESET_POSITION) >= POSITION_THRESHOLD) {
                double newPosition = Math.min(RESET_POSITION, robot.intakeRotation.getPosition() + POSITION_INCREMENT);
                robot.intakeRotation.setPosition(newPosition);
            } else {
                robot.intakeRotation.setPosition(ZERO_POSITION);
            }
        }


        //Swetha's Controls:

        //Extendo Slides Stuff:
        if (!extendoManualControl) {
            robot.extendoSubsystem.extendoSlidesLoop();
        } else {
            robot.extendoSubsystem.extendoManualControlLoop(swethaController.left_stick_x);
        }

        if (swethaController.right_stick_y > 0) {
            extendoManualControl = true;
        } else if (swethaController.square || swethaController.circle || swethaController.dpad_right || swethaController.dpad_left) {
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
            robot.depositSubsystem.outtakeSlidesLoop();
        } else {
            robot.depositSubsystem.depositManualControlLoop(swethaController.right_stick_y);
        }

        if (swethaController.right_stick_y > 0) {
            depositManualControl = true;
        } else if (swethaController.triangle || swethaController.cross || swethaController.dpad_up || swethaController.dpad_down) {
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
            schedule (
                    new AllSystemInitializeCommand(robot)
            );
        }

    }
}
