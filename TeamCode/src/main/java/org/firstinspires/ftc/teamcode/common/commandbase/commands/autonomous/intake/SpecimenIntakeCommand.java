package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenIntakeCommand extends SequentialCommandGroup {

    public SpecimenIntakeCommand(RobotHardware robot, double intakeRotation, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.SPECIMEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.SPECIMEN)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.SPECIMEN)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, intakeRotation)),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(extendoPosition))
                )
        );
    }
}