package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class BucketDropCommand extends SequentialCommandGroup {

    public BucketDropCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.DUNK)),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawOpen()),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING))
                )
        );
    }

}