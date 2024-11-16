package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class BucketDropCommand extends SequentialCommandGroup {
    public BucketDropCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmDunk()),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawOpen()),
                new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmRaising())
        );
    }
}