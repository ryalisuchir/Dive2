package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakeCommand extends ParallelCommandGroup {
    public IntakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                    new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                    new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.INTAKE)),
                    new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.CoaxialState.REST)),
                    new InstantCommand(() -> robot.intakeRotationSubsystem.updateRotation(null)),
                    new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(Globals.EXTENDO_MAX_EXTENSION)),
                    new WaitCommand(0)
                    )
        );
    }
}