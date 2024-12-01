package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TOuttakeCommand extends SequentialCommandGroup {
    public TOuttakeCommand(
            RobotHardware robot,
            double liftPositionInput,
            double outtakeArmInput,
            double outtakeRotationInput
    ) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmCustom(outtakeArmInput)),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(liftPositionInput)),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.outtakeRotationCustom(outtakeRotationInput))
                )
        );
    }
}
