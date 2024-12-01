package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.wall;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.teleop.TeleOpGlobals;

public class TSpecimenTransferCommand extends SequentialCommandGroup {
    public TSpecimenTransferCommand(
            RobotHardware robot,
            long waitMillisecondsInput
    ) {
        super(
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed()),
                new WaitCommand(waitMillisecondsInput),
                new OuttakeCommand(robot, TeleOpGlobals.LIFT_SPECIMEN_POS)
        );
    }
}

