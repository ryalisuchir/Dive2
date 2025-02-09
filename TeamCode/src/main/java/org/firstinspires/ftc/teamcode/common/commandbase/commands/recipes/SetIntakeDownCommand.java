package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeCoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeRotationCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SetIntakeDownCommand extends SequentialCommandGroup {

    public SetIntakeDownCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new IntakeClawCommand(robot.intakeClawSubsystem, Globals.IntakeClawState.OPEN),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.SCANNING),
                        new IntakeCoaxialCommand(robot.intakeCoaxialSubsystem, Globals.IntakeCoaxialState.INTAKE),
                        new IntakeRotationCommand(robot.intakeRotationSubsystem, Globals.IntakeRotationState.REST),
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                )
        );
    }

}