package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class DirectTransferCommand extends SequentialCommandGroup {
    public DirectTransferCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION),
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.CLOSED)),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.TRANSFER),
//                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.BETWEEN)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.TRANSFER)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.TRANSFER, 0.28)),
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN),
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER)
                )
        );
    }

}

