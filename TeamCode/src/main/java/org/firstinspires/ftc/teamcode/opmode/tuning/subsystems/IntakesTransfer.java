package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.HighBucketOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Photon
@TeleOp
public class IntakesTransfer extends CommandOpMode {
    private RobotHardware robot;
    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
    }

    @Override
    public void run() {
        super.run();

        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.depositSubsystem.outtakeSlidesLoop();

        boolean triangle = gamepad1.triangle;
        if (triangle) {
            schedule (
                    new AllSystemInitializeCommand(robot)
            );
        }

        boolean circle = gamepad1.circle;
        if (circle) {
            schedule(
                    new IntakeCommand(robot, 0.5, 500)
            );
        }

        boolean square = gamepad1.square;
        if (square) {
            schedule(
                    new ScanningCommand(robot, 0.5, 0)
            );
        }

        boolean x = gamepad1.cross;
        if (x) {
            schedule(
                    new TransferCommand(robot)
            );
        }

        boolean up = gamepad1.dpad_up;
        if (up) {
            schedule (
                    new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen())
            );
        }
        boolean down = gamepad1.dpad_down;
        if (down) {
            schedule (
                    new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed())
            );
        }

        boolean left = gamepad1.dpad_left;
        if (left) {
            schedule (
                    new HighBucketOuttakeCommand(robot)
            );
        }


    }
}
