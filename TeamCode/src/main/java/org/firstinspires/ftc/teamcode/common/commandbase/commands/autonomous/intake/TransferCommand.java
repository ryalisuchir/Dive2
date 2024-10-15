package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.depositSubsystem.outtakeRetract()),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmTransfer()),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeTransfer()),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.outtakeRotationTransfer()),

                        new InstantCommand(() -> robot.intakeRotationSubsystem.updateRotation(0.5)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.TRANSFER))
                ),
                new WaitCommand(0),
                new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmTransfer())
        );
    }
}

