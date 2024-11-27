package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TIntakeCommand extends SequentialCommandGroup {

    public TIntakeCommand(
            RobotHardware robot,
            double intakeClawInput,
            double intakeFourBarInput,
            double intakeCoaxialInput,
            double intakeRotationInput,
            double intakeExtendoInput
    ) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawCustom(intakeClawInput)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarCustom(intakeFourBarInput)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.intakeCoaxialCustom(intakeCoaxialInput)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.intakeRotationCustom(intakeRotationInput)),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(intakeExtendoInput))
                        )
                )
        );
    }
}