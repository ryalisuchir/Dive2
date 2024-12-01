package org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class TSpecimenIntakeCommand extends SequentialCommandGroup {
    public TSpecimenIntakeCommand(
            RobotHardware robot,
            double outtakeArmPositionInput
    ) {
        super(
                new ParallelCommandGroup(
                        new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                        new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.SPECIMEN_INTAKE))
                ),
                new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawOpen())
        );
    }
}