package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.slides.DepositSlidesCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class OuttakeCommand extends SequentialCommandGroup {

    public OuttakeCommand(RobotHardware robot, double liftPositionInput) {
        super(getCommands(robot, liftPositionInput));
    }

    public static SequentialCommandGroup getCommands(RobotHardware robot, double liftPositionInput) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        if (liftPositionInput == Globals.LIFT_HIGH_POS) {
            commandGroup.addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING)),
                            new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_HIGH_POS),
                            new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.EXIT))
                    )
            );
        } else if (liftPositionInput == Globals.LIFT_MID_POS) {
            commandGroup.addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING)),
                            new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_MID_POS),
                            new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.EXIT))
                    )
            );
        } else if (liftPositionInput == Globals.LIFT_SPECIMEN_POS) {
            commandGroup.addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.SPECIMEN)),
                            new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_SPECIMEN_POS),
                            new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.SPECIMEN))
                    )
            );
        } else if (liftPositionInput == Globals.LIFT_RETRACT_POS) {
            commandGroup.addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING)),
                            new DepositSlidesCommand(robot.depositSubsystem, Globals.LIFT_RETRACT_POS),
                            new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.EXIT))
                    )
            );
        }

        return commandGroup;
    }

}
