package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CloseAndTransferCommand extends SequentialCommandGroup {
    public CloseAndTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.INTAKE),
                new WaitCommand(150),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                new WaitCommand(350),
                new TransferCommand(robot),
                new WaitCommand(350),
                new ClawTransferCommand(robot)
                )
        );
    }

}

