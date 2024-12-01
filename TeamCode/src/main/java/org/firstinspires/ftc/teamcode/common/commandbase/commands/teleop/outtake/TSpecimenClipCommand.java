package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TSpecimenClipCommand extends SequentialCommandGroup {
    public TSpecimenClipCommand(
            RobotHardware robot,
            double amountLowered
    ) {
        super(
                new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_RETRACT_POS - amountLowered)),
                new WaitCommand(150),
                new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.OPEN)),
                new OuttakeTransferReadyCommand(robot)
        );
    }
}