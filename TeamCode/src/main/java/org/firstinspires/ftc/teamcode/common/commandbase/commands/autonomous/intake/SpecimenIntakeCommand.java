package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenIntakeCommand extends SequentialCommandGroup {
    public SpecimenIntakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.depositSubsystem.outtakeRetract()),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.SPECIMEN_INTAKE)),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.SPECIMEN))
                ),
                new WaitCommand(100),
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawOpen())
        );
    }
}