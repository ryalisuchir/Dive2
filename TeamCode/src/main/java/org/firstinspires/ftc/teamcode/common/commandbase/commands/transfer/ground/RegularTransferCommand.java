package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RegularTransferCommand extends SequentialCommandGroup {
    public RegularTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new TransferCommand(robot)
                                )
                        ),
                        new WaitCommand(275),
                        new ClawTransferCommand(robot)
                )
        );
    }

}
