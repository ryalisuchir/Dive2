package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.TransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class RetractedTransferCommand extends SequentialCommandGroup {
    public RetractedTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new IntakeClawCommand(robot.intakeClawSubsystem, Globals.IntakeClawState.OPEN_TRANSFER),
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(50),
                                        new TransferCommand(robot)
                                )
                        ),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new ClawTransferCommand(robot)
                        )
                )
        );
    }

}

