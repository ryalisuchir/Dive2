package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.wall;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;

public class SpecimenGrabAndTransferAndLiftCommand extends SequentialCommandGroup {
    public SpecimenGrabAndTransferAndLiftCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed()),
                new WaitCommand(350),
                new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
        );
    }
}

