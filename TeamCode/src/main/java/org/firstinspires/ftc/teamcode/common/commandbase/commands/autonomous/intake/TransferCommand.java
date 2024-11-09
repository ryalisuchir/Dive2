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
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarTransfer()),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.coaxialTransfer()),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.intakeRotationTransfer()),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(0)),
                        new WaitCommand(0)
                ),
                new WaitCommand(0)
//                new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmTransfer())
        );
    }
}

