package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenClipCommand extends SequentialCommandGroup {
    public SpecimenClipCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_RETRACT_POS - Globals.LIFT_SPECIMEN_DROP)),
                new WaitCommand(150),
                new InstantCommand(() -> robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.OPEN)),
                new OuttakeTransferReadyCommand(robot),
                        new WaitCommand(500)
                )
                );
    }
}