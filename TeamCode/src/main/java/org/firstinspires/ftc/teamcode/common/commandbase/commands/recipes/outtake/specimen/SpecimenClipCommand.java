package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenClipCommand extends SequentialCommandGroup {
    public SpecimenClipCommand(RobotHardware robot) {
        super(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_OUTTAKE),
//                                new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_SPECIMEN_POS - (Globals.LIFT_SPECIMEN_DROP)),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(100),
//                                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN)
//                                )
//                        ),
//                        new WaitCommand(120),
//                        new ParallelCommandGroup(
//                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING),
//                                new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS)
//                        )
//                )
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_SMASH),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN)
                                )
                        )
        );
    }
}