package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;

public class SecondarySpecimenClipCommand extends SequentialCommandGroup {
    public SecondarySpecimenClipCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_OUTTAKE),
                                new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_SPECIMEN_POS - (Globals.LIFT_SPECIMEN_DROP + 100))
                        ),
                        new WaitCommand(400),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN)
                )
        );
    }
}