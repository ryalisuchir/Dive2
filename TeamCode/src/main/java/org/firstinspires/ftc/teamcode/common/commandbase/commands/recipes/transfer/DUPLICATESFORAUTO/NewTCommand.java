package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.DUPLICATESFORAUTO;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeRotationCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class NewTCommand extends SequentialCommandGroup {
    public NewTCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.TRANSFER),
                        new InstantCommand(() -> {
                            robot.intakeCoaxialSubsystem.skibidi(0.68);
                        }),
//                        new IntakeCoaxialCommand(robot.intakeCoaxialSubsystem, Globals.IntakeCoaxialState.TRANSFER),
                        new IntakeRotationCommand(robot.intakeRotationSubsystem, Globals.INTAKE_ROTATION_TRANSFER)
                )
        );
    }

}

