package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CloseAndTransferCommand extends SequentialCommandGroup {
    public CloseAndTransferCommand(RobotHardware robot) {
        super(
                new IntakePeckerCommand(robot, robot.intakeRotation.getPosition(), robot.extendoMotor.getCurrentPosition()),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                new WaitCommand(350),
                new TransferCommand(robot),
                new WaitCommand(750),
                new ClawTransferCommand(robot)
        );
    }
}

