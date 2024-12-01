package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TBucketDropCommand extends SequentialCommandGroup {
    public TBucketDropCommand(
            RobotHardware robot,
            double outtakeArmInput,
            double outtakeClawInput
    ) {
        super(
                new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmCustom(outtakeArmInput)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawCustom(outtakeClawInput)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING))
        );
    }
}