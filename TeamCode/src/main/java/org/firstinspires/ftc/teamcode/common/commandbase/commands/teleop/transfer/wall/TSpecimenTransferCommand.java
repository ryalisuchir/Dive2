package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.wall;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TSpecimenTransferCommand extends SequentialCommandGroup {
    public TSpecimenTransferCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed()),
                new WaitCommand(100),
                new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING))
        );
    }
}

