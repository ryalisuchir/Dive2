package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class LigmaTransferCommand extends SequentialCommandGroup {
    public LigmaTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new OuttakeTransferReadyCommand(robot),
                        new WaitCommand(10),
                        new TransferCommand(robot),
                        new WaitCommand(450),
                        new ClawTransferCommand(robot)
                )
        );
    }

}
