package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleopspecific;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class CustomOuttakeCommand extends SequentialCommandGroup {

    public CustomOuttakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_HIGH_POS),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.BUCKET)
                        )
                )
        );
    }

}