package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakePeckerCommand extends SequentialCommandGroup {
    public IntakePeckerCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.INTAKE),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawClosed()),
                        new WaitCommand(100),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.SCANNING)
                )
        );
    }

}