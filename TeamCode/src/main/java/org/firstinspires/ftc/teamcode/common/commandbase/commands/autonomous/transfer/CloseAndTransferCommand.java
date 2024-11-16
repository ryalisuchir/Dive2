package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CloseAndTransferCommand extends SequentialCommandGroup {
    public CloseAndTransferCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                new TransferCommand(robot)
        );
    }
}

