package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class AllSystemRestCommand extends SequentialCommandGroup {

    public AllSystemRestCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.RESTING)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.REST)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, 0.5)),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(0)),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeRetract()),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.OPEN_TRANSFER)),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.TRANSFER)),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.TRANSFER))
                )
        );
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}