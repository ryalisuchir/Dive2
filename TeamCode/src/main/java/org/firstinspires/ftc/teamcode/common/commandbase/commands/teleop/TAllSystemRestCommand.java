package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TAllSystemRestCommand extends SequentialCommandGroup {

    public TAllSystemRestCommand(
            RobotHardware robot,
            double intakeClawInput,
            double intakefourBarInput,
            double intakeCoaxialInput,
            double intakeRotationInput,
            double extendoPositionInput,
            double depositPositionInput,
            double outtakeClawInput,
            double outtakeArmInput
    ) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawCustom(intakeClawInput)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarCustom(intakefourBarInput)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.intakeCoaxialCustom(intakeCoaxialInput)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.intakeRotationCustom(intakeRotationInput)),
                        new InstantCommand(() -> robot.extendoSubsystem.extendoSetPosition(extendoPositionInput)),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(depositPositionInput)),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawCustom(outtakeClawInput)),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmCustom(outtakeArmInput))
                )
        );
    }
}