package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakePeckerCommand extends SequentialCommandGroup {
    public IntakePeckerCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.INTAKE),
                        new WaitCommand(80),
                        new IntakeClawCommand(robot.intakeClawSubsystem, Globals.IntakeClawState.CLOSED),
                        new WaitCommand(125),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.SCANNING)
                )
        );
    }
}