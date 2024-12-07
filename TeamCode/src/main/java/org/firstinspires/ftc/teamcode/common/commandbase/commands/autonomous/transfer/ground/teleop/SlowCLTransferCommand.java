package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;

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

