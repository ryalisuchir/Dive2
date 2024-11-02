package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakeCommand extends ParallelCommandGroup {
    public IntakeCommand(RobotHardware robot, double intakeRotation, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.INTAKE)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.CoaxialState.REST)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.updateRotation(intakeRotation)),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(extendoPosition)),
                        new WaitCommand(0)
                )
        );
    }
}
