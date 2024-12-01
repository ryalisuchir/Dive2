package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.Intake4BarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;

public class ClawTransferCommand extends SequentialCommandGroup {
    public ClawTransferCommand(RobotHardware robot) {
        super(
                new Intake4BarCommand(robot.intake4BarSubsystem, Globals.FourBarState.TRANSFER),
                new WaitCommand(100),
                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.CLOSED),
                new WaitCommand(300),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                new ParallelCommandGroup(
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING),
                        new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, 0)
                )
        );
    }

}

