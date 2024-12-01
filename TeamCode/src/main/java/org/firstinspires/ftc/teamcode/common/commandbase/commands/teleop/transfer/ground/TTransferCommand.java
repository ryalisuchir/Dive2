package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TTransferCommand extends SequentialCommandGroup {
    public TTransferCommand(
            RobotHardware robot,
            double intakeFourBarInput
    ) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                        new WaitCommand(350),
                        new TransferCommand(robot),
                        new WaitCommand(750),
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarCustom(intakeFourBarInput)),
                        new WaitCommand(100),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed()),
                        new WaitCommand(75),
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING)),
                                new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.BETWEEN))
                        )
                )
        );
    }
}

