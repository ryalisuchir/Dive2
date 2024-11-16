package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.extendoSubsystem.extendoRetract()),
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.CLOSED)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.BETWEEN)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.TRANSFER)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.TRANSFER, 0)),
//
                        new InstantCommand(() -> robot.depositSubsystem.outtakeRetract()),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.OPEN)),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.TRANSFER)),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.TRANSFER))
                )
//                new WaitCommand(0),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
//                        new ParallelCommandGroup(
//                         new WaitCommand(50),
//                         new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.CLOSED))
//                        )
//                ),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmRaising()),
//                        new ScanningCommand(robot, 0.5, 0)
//                )
        );
    }
}

