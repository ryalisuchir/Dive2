package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.custom;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CustomLowBucketCommand extends SequentialCommandGroup { // TeleOp Specific Command

    public CustomLowBucketCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_MID_POS),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.BUCKET)
                        )
                )
        );
    }

}