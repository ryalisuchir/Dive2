package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand1;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Photon
@TeleOp
public class Intakes extends CommandOpMode {
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

        boolean circle = gamepad1.circle;
        if (circle) {
            schedule(
                    new IntakeCommand1(robot, 500)
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

    }
}
