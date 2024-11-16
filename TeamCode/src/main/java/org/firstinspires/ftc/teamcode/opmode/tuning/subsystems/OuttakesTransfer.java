package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.HighBucketOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.SpecimenOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class OuttakesTransfer extends CommandOpMode {
    private RobotHardware robot;
    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.depositSubsystem.outtakeSlidesLoop();

        boolean triangle = gamepad1.triangle;
        if (triangle) {
            schedule (
                    new OuttakeTransferReadyCommand(robot)
            );
        }

        boolean circle = gamepad1.circle;
        if (circle) {
            schedule(
                    new BucketDropCommand(robot)
            );
        }

        boolean square = gamepad1.square;
        if (square) {
            schedule(
                    new SpecimenOuttakeCommand(robot)
            );
        }

        boolean up = gamepad1.dpad_up;
        if (up) {
            schedule (
                    new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawOpen())
            );
        }
        boolean down = gamepad1.dpad_down;
        if (down) {
            schedule (
                    new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed())
            );
        }

    }
}
