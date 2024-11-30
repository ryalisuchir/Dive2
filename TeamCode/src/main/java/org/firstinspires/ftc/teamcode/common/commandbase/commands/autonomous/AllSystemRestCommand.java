package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class AllSystemRestCommand extends SequentialCommandGroup {

    public AllSystemRestCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.RESTING)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.REST)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, 0.39)),
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION),
                        new InstantCommand(() -> robot.depositSubsystem.outtakeRetract()),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.TRANSFER)),
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER)
                )
        );
    }

}