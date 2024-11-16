package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ClawTransferCommand extends SequentialCommandGroup {
    public ClawTransferCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.intake4BarSubsystem.update(Globals.FourBarState.TRANSFER)),
                new WaitCommand(100),
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed()),
                new WaitCommand(100),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeArmSubsystem.outtakeArmRaising()),
                        new ScanningCommand(robot, 0.5, 0)
                )
        );
    }
}

