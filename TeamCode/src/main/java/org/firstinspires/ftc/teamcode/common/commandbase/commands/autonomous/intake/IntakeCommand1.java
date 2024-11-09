package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakeCommand1 extends ParallelCommandGroup {

    public IntakeCommand1(RobotHardware robot, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarIntake()),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.coaxialIntake()),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.intakeRotationAuto1()),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(extendoPosition)),
                        new WaitCommand(0)
                )
        );
    }
}