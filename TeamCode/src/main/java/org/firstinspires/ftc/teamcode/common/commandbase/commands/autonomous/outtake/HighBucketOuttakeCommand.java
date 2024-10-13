package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class HighBucketOuttakeCommand extends SequentialCommandGroup {
    public HighBucketOuttakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeGrab()),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmHighest()),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeMaxExtend()),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.outtakeRotationExit())
                )
        );
    }
}
