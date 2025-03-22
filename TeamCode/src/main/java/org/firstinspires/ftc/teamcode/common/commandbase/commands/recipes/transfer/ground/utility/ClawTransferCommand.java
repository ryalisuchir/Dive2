package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ClawTransferCommand extends SequentialCommandGroup {
    public ClawTransferCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.CLOSED),
                        new WaitCommand(10),
                        new IntakeClawCommand(robot.intakeClawSubsystem, Globals.IntakeClawState.OPEN),
                        new WaitCommand(70),
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING),
                                new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION)
                        )
                )
        );
    }

}

