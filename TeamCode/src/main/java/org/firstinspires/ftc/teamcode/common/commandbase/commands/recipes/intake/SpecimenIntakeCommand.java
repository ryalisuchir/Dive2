package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenIntakeCommand extends SequentialCommandGroup {
    public SpecimenIntakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_INTAKE)
                ),
                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN)
        );
    }
}