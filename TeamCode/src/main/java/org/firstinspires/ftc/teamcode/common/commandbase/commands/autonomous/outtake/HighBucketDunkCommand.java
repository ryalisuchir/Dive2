package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class HighBucketDunkCommand extends SequentialCommandGroup {
    public HighBucketDunkCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmExit()),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeRelease())
                )
        );
    }
}
