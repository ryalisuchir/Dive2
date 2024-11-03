package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenOuttakeCommand extends SequentialCommandGroup {
    public SpecimenOuttakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.CLOSED)),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmExit()),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeSpecimenExtend()),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.SPECIMEN))
                )
        );
    }
}