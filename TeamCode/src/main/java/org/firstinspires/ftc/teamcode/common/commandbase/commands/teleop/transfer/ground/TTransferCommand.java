package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake.TScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.ExtendoSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.teleop.TeleOpGlobals;

public class TTransferCommand extends SequentialCommandGroup {
    public TTransferCommand(
            RobotHardware robot,
            double intakeFourBarInput,
            double coaxialInput
    ) {
        super(
                //Transfer Command:
                new ParallelCommandGroup(
                        new ExtendoSlidesCommand(robot.extendoSubsystem, TeleOpGlobals.EXTENDO_MAX_RETRACTION),
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.CLOSED)),
                        new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.BETWEEN),
                        new InstantCommand(() -> robot.intakeCoaxialSubsystem.intakeCoaxialCustom(coaxialInput)),
                        new InstantCommand(() -> robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.TRANSFER, Globals.INTAKE_ROTATION_REST)),
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN),
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER)
                ),
                new WaitCommand(50),
                //Claw Transfer Command:
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake4BarSubsystem.intake4BarCustom(intakeFourBarInput)),
                        new WaitCommand(100),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(300),
                        new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING),
                                new TScanningCommand(
                                        robot,
                                        TeleOpGlobals.INTAKE_CLAW_OPEN,
                                        TeleOpGlobals.INTAKE_FOURBAR_SCANNING,
                                        TeleOpGlobals.INTAKE_COAXIAL_INTAKE,
                                        TeleOpGlobals.INTAKE_ROTATION_REST,
                                        TeleOpGlobals.EXTENDO_MAX_RETRACTION
                                )
                        )
                )
        );
    }
}

