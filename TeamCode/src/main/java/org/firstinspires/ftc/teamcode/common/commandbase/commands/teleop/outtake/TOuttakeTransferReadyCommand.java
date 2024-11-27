package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TOuttakeTransferReadyCommand extends SequentialCommandGroup {
    public TOuttakeTransferReadyCommand(
            RobotHardware robot,
            double outtakeArmInput,
            double depositSlidesInput,
            double outtakeRotationInput,
            double outtakeClawInput
    ) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmCustom(outtakeArmInput)),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(depositSlidesInput)),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.outtakeRotationCustom(outtakeRotationInput)),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawCustom(outtakeClawInput))
                )
        );
    }
}
