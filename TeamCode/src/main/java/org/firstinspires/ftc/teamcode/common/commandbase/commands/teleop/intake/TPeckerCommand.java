package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TPeckerCommand extends SequentialCommandGroup {

    public TPeckerCommand(
            RobotHardware robot,
            double intakeFourBarInput
    ) {
        super(
                new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarCustom(intakeFourBarInput)),
                new WaitCommand(150),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                new WaitCommand(150),
                new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.SCANNING))

        );
    }
}