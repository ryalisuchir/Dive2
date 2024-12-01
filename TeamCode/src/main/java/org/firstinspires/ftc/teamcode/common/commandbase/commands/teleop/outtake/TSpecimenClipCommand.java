package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.outtake;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.auto.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.teleop.TeleOpGlobals;

public class TSpecimenClipCommand extends SequentialCommandGroup {
    public TSpecimenClipCommand(
            RobotHardware robot,
            double amountLowered
    ) {
        super(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.SPECIMEN_OUTTAKE),
                                new DepositSlidesCommand(robot.depositSubsystem, TeleOpGlobals.LIFT_SPECIMEN_POS - amountLowered)
                        ),
                        new WaitCommand(200),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN)
                )
        );
    }
}