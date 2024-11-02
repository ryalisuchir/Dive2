package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Photon
@TeleOp
public class Intakes extends CommandOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    double speed;
    @Override
    public void initialize() {
        robot.init(hardwareMap, Globals.DEFAULT_START_POSE);
    }

    @Override
    public void run() {
        super.run();
//        robot.extendoSubsystem.currentLoop();
//        robot.extendoSubsystem.extendoUpdate();

        boolean circle = gamepad1.circle;
        if (circle) {
            schedule(
                    new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN))
            );
        }
        boolean x = gamepad1.x;
        if (x) {
            schedule(
                    new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.CLOSED))
            );
        }

        boolean triangle = gamepad1.triangle;
        if (triangle) {
            schedule(
                    new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.INTAKE))
            );
        }

        boolean square = gamepad1.square;
        if (square) {
            schedule(
                    new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.RESTING))
            );
        }

        boolean dpad_up = gamepad1.dpad_up;
        if (dpad_up) {
            schedule(
                    new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.TRANSFER))
            );
        }

        boolean dpad_down = gamepad1.dpad_down;
        if (dpad_down) {
            schedule(
                    new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.SCANNING))
            );
        }

        boolean dpad_left = gamepad1.dpad_left;
        if (dpad_left) {
            schedule(
                    new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.LOW))
            );
        }

        boolean dpad_right = gamepad1.dpad_right;
        if (dpad_right) {
            schedule(
                    new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.CoaxialState.REST))
            );
        }

    }
}
