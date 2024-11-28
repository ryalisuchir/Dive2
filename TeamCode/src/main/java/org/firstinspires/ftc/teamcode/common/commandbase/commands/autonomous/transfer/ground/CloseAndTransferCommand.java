package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CloseAndTransferCommand extends SequentialCommandGroup {
    public CloseAndTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.INTAKE)),
                new WaitCommand(150),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                new WaitCommand(350),
                new TransferCommand(robot),
                new WaitCommand(750),
                new ClawTransferCommand(robot),
                        new WaitCommand(1300)
                )
        );
    }

}

