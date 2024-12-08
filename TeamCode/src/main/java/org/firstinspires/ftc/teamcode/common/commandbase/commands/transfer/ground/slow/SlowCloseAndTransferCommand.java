package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.slow;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SlowCloseAndTransferCommand extends SequentialCommandGroup {
    public SlowCloseAndTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.INTAKE),
                        new WaitCommand(150),
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                        new WaitCommand(350),
                        new TransferCommand(robot),
                        new WaitCommand(1500),
                        new ClawTransferCommand(robot)
                )
        );
    }

}

