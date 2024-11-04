package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand1;
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
//        robot.extendoSubsystem.currentLoop();
//        robot.extendoSubsystem.extendoUpdate();

        boolean circle = gamepad1.circle;
        if (circle) {
            schedule(
                    new IntakeCommand1(robot, 0)
            );
        }
        boolean x = gamepad1.x;
        if (x) {
            schedule(
                    new TransferCommand(robot)
            );
        }

    }
}
