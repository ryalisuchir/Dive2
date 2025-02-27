package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class BucketDropCommand extends SequentialCommandGroup {

    public BucketDropCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.BUCKET),
                        new WaitCommand(75),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN),
                        new WaitCommand(75),
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.RAISING)
                )
        );
    }

}