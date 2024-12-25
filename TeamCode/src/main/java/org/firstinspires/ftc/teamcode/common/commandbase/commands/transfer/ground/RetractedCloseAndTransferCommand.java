package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RetractedCloseAndTransferCommand extends SequentialCommandGroup {
    public RetractedCloseAndTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new OuttakeTransferReadyCommand(robot),
                        new WaitCommand(350),
                        new TransferCommand(robot),
                        new WaitCommand(1500),
                        new ClawTransferCommand(robot)
                )
        );
    }

}

