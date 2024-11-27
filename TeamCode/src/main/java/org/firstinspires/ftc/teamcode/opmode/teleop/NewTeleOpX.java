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
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.TAllSystemRestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake.TPeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.ground.TTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class NewTeleOpX extends CommandOpMode {
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean extendoManualControl;
    private boolean driverControlUnlocked;

    Gamepad ahnafController, swethaController;
    GamepadEx ahnafButtonController, swethaButtonController;

    double robotPitch;
    double antiTipPower;

    @Override
    public void initialize() {
        //Do not remove:
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE); //Registers subsystems too

        //Setting controllers:
        ahnafController = gamepad1;
        swethaController = gamepad2;
        ahnafButtonController = new GamepadEx(gamepad1);
        swethaButtonController = new GamepadEx(gamepad2);

        //All manual controls:
        driverControlUnlocked = true;
        depositManualControl = true;
        extendoManualControl = true;

        //Ahnaf Controls:

        ahnafButtonController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
          new TPeckerCommand(
                  robot,
                  Globals.INTAKE_FOURBAR_LOW //fourbar position for the drop as it grabs
          ));


        ahnafButtonController.getGamepadButton(GamepadKeys.Button.A).whenPressed( //x
                new TTransferCommand(
                    robot,
                    Globals.INTAKE_FOURBAR_TRANSFER
                ));

        }

    @Override
    public void run() {

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.currentLoop();


        if (driverControlUnlocked) {
            robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.5 * Math.tan(1.12 * ahnafController.left_stick_y),
                            0.5 * Math.tan(1.12 * ahnafController.left_stick_x)
                    ),
                    -ahnafController.right_stick_x //Making turns normal again
            ));
        }

        if (ahnafController.ps) {
            ahnafController.rumble(1000);
            swethaController.rumble(1000);
            schedule (
                    new TAllSystemRestCommand(
                            robot,
                            Globals.INTAKE_CLAW_OPEN, //intakeClawInput
                            Globals.INTAKE_FOURBAR_RESTING, //intakeFourBarInput
                            Globals.INTAKE_COAXIAL_RESTING, //intakeCoaxialInput
                            0.5, //intakeRotationInput
                            0, //extendoPositionInput
                            0, //depositPositionInput
                            Globals.OUTTAKE_CLAW_OPEN, //outtakeClawInput
                            Globals.OUTTAKE_ROTATION_TRANSFER, //outtakeRotationInput
                            Globals.OUTTAKE_ARM_TRANSFER //outtakeArmInput
                    )
            );
        }

    }
}
