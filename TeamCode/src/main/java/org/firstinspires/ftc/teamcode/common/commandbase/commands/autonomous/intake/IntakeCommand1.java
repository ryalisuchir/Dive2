package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class IntakeCommand1 extends ParallelCommandGroup {
    public IntakeCommand1(RobotHardware robot, double intakeRotation, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.INTAKE)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.REST)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.AUTO_1)),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(extendoPosition)),
                        new WaitCommand(0)
                )
        );
    }
}
