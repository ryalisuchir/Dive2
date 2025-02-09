package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeCoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeRotationCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CameraScanningPositionCommand extends SequentialCommandGroup {
    public CameraScanningPositionCommand(RobotHardware robot, double intakeRotation, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new IntakeClawCommand(robot.intakeClawSubsystem, Globals.IntakeClawState.OPEN),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.CAMERA_READING),
                        new IntakeCoaxialCommand(robot.intakeCoaxialSubsystem, Globals.IntakeCoaxialState.CAMERA_READING),
                        new IntakeRotationCommand(robot.intakeRotationSubsystem, Globals.IntakeRotationState.REST),
                        //Prevents intake stuff from getting stuck:
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, extendoPosition)
                        )
                )
        );
    }
}
