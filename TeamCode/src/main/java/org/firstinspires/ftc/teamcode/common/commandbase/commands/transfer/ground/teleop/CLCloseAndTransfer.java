package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.DirectTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CLCloseAndTransfer extends SequentialCommandGroup {
    public CLCloseAndTransfer(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new OuttakeTransferReadyCommand(robot),
                        new WaitCommand(350),
                        new TransferCommand(robot),
                        new WaitCommand(500),
                        new ClawTransferCommand(robot)
                )
        );
    }

}

