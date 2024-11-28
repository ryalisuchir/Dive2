package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION),
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.CLOSED)),
                        new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.BETWEEN)),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.TRANSFER)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.TRANSFER, 0)),
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN),
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER)
                )
        );
    }

}

