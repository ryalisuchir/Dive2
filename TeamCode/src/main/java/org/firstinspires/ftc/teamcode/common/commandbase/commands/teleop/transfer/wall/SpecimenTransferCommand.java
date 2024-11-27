package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.wall;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenTransferCommand extends SequentialCommandGroup {
    public SpecimenTransferCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                new WaitCommand(350),
                new TransferCommand(robot),
                new WaitCommand(750),
                new ClawTransferCommand(robot)
        );
    }
}

