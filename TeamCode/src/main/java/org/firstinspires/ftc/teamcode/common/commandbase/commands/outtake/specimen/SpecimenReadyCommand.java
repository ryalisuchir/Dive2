package org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SpecimenReadyCommand extends SequentialCommandGroup {
    public SpecimenReadyCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_INTAKE),
                                new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                        )
                )
        );
    }

}
