package org.firstinspires.ftc.teamcode.recipes.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.UninterruptableCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;

@TeleOp
public class TeleOpR extends CommandOpMode {
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean extendoManualControl;
    private boolean driverControlUnlocked;
    public static final double[] intakeRotationPositions = { 0.83, 0.7383, 0.64667, 0.555, 0.46333, 0.37167, 0.28 };
    Gamepad ahnafController, swethaController;
    GamepadEx ahnafLigmaController, swethaLigmaController;
    Gamepad ahnafPreviousGamepad = new Gamepad();

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
        extendoManualControl = true;

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

        ahnafLigmaController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, robot.extendoMotor.getCurrentPosition())
        );

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
                            0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
                            0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
                    ),
                    -ahnafController.right_stick_x
            ));
        }


        //Swetha's Controls:
        //Extendo Slides Stuff:

        robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);

        if (ahnafController.cross) {
            schedule(
                    new UninterruptableCommand(new CloseAndTransferCommand(robot))
            );
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
            robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


    }
}
