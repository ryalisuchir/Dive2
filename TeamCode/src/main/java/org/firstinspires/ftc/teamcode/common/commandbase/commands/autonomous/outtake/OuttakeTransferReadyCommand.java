package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.regular.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.regular.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.maincommandbase.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class OuttakeTransferReadyCommand extends SequentialCommandGroup {
    public OuttakeTransferReadyCommand(RobotHardware robot) {
        super(
                new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new OuttakeArmCommand(robot.outtakeArmSubsystem, Globals.OuttakeArmState.TRANSFER),
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new OuttakeClawCommand(robot.outtakeClawSubsystem, Globals.OuttakeClawState.OPEN_TRANSFER)
                )
                )
        );
    }

}
