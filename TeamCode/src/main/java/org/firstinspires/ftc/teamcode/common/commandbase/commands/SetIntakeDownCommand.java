package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SetIntakeDownCommand extends SequentialCommandGroup {

    public SetIntakeDownCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.SCANNING)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.INTAKE)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, Globals.INTAKE_ROTATION_REST)),
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION)
                )
        );
    }

}