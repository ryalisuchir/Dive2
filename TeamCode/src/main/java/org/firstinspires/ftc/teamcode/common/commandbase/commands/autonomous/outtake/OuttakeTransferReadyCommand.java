package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class OuttakeTransferReadyCommand extends SequentialCommandGroup {
    public OuttakeTransferReadyCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.TRANSFER)),
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.TRANSFER)),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.OPEN_TRANSFER))
                )
                )
        );
    }

}
