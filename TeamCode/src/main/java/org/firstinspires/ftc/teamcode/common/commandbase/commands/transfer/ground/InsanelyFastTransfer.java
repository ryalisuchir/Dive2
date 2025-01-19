package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class InsanelyFastTransfer extends SequentialCommandGroup {
    public InsanelyFastTransfer(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new TransferCommand(robot)
                                )
                        ),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new ClawTransferCommand(robot)
                        )
                )
        );
    }

}
