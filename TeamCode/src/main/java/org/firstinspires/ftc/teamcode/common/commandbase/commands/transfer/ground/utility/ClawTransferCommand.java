package org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ClawTransferCommand extends SequentialCommandGroup {
    public ClawTransferCommand(RobotHardware robot) {
        super(
                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.CLOSED),
                new WaitCommand(75),
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()),
                new WaitCommand(75),
                new ParallelCommandGroup(
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING),
                        new ScanningCommand(robot, Globals.INTAKE_ROTATION_REST, 0)
                )
        );
    }

}

