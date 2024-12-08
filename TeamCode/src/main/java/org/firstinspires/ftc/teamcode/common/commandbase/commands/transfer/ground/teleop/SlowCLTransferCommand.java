package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SlowCLTransferCommand extends SequentialCommandGroup {
    public SlowCLTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new OuttakeTransferReadyCommand(robot),
                        new WaitCommand(350),
                        new TransferCommand(robot),
                        new WaitCommand(2000),
                        new ClawTransferCommand(robot)
                )
        );
    }

}

