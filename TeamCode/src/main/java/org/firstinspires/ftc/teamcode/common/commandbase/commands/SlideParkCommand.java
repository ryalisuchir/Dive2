package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SlideParkCommand extends SequentialCommandGroup {
    public SlideParkCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER),
                                new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_PARK_POS),
                                new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                        )
                )
        );
    }

}
