package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenReadyCommand extends SequentialCommandGroup {
    public SpecimenReadyCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_INTAKE),
                                new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN)
                        )
                )
        );
    }

}
