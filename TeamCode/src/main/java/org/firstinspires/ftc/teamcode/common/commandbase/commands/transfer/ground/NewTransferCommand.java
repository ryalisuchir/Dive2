package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class NewTransferCommand extends SequentialCommandGroup {
    public NewTransferCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new OuttakeTransferReadyCommand(robot),
                        new WaitCommand(300),
                        new ParallelCommandGroup( //TransferCommand
                                new ExtendoSlidesCommand(robot.extendoSubsystem, Globals.EXTENDO_MAX_RETRACTION),
                                new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN_TRANSFER)),
                                new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.TRANSFER),
                                new InstantCommand(() -> robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.TRANSFER)),
                                new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.CUSTOM, Globals.INTAKE_ROTATION_TRANSFER))
                        )
                ),
                new WaitCommand(75),
                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.CLOSED),
                new WaitCommand(75),
                new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN)),
                new ParallelCommandGroup(
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING),
                        new ScanningCommand(robot, Globals.INTAKE_ROTATION_TRANSFER, Globals.EXTENDO_MAX_RETRACTION)
                )
        );
    }

}
