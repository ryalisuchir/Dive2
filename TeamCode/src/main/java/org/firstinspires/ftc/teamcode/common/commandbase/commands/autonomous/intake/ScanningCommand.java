package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;

public class ScanningCommand extends SequentialCommandGroup {
    public ScanningCommand(RobotHardware robot, double intakeRotation, double extendoPosition) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.SCANNING),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.INTAKE)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, intakeRotation)),
                        //Prevents intake stuff from getting stuck:
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                new ExtendoSlidesCommand(robot.extendoSubsystem, extendoPosition)
                        )
                )
        );
    }
}
