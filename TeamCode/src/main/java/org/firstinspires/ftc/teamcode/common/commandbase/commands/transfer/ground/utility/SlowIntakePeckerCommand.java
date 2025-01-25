package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SlowIntakePeckerCommand extends SequentialCommandGroup {
    public SlowIntakePeckerCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarCustom(0.55)),
                        new WaitCommand(200),
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                        new WaitCommand(100),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.SCANNING)
                )
        );
    }

}