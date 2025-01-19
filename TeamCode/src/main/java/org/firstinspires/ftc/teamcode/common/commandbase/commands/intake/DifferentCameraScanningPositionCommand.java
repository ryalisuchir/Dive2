package org.firstinspires.ftc.teamcode.common.commandbase.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class DifferentCameraScanningPositionCommand extends SequentialCommandGroup {
    public DifferentCameraScanningPositionCommand(RobotHardware robot, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.CAMERA_READING),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.intakeCoaxialCustom(0.35))),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, Globals.INTAKE_ROTATION_TRANSFER)),
                        //Prevents intake stuff from getting stuck:
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, extendoPosition)
                        )
        );
    }
}
