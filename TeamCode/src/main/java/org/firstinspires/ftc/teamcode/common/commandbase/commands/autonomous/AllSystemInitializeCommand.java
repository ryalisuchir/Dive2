package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class AllSystemInitializeCommand extends SequentialCommandGroup {

    public AllSystemInitializeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.RESTING)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.REST)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, 0.5)),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(0)),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_RETRACT_POS)),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.CLOSED)),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.SPECIMEN)),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.SPECIMEN))
                )
        );
    }
}