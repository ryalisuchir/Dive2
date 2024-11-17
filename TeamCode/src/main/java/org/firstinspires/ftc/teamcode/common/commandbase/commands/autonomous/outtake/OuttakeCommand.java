package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(RobotHardware robot, double liftPositionInput) {
        super(
          getCommands(robot, liftPositionInput)
        );
    }

    private static Command[] getCommands(RobotHardware robot, double liftPositionInput) {
        if (liftPositionInput == Globals.LIFT_HIGH_POS) {
            return new Command[]{
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING)),
                            new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_HIGH_POS)),
                            new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.EXIT))
                    )
            };
        } else if (liftPositionInput == Globals.LIFT_MID_POS) {
            return new Command[]{
                    new ParallelCommandGroup(
                    new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.RAISING)),
                    new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_MID_POS)),
                    new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.EXIT))
                    )
            };
        } else if (liftPositionInput == Globals.LIFT_SPECIMEN_POS) {
            return new Command[]{
                    new SequentialCommandGroup(
                    new InstantCommand(() -> robot.outtakeClawSubsystem.outtakeClawClosed()),
                    new WaitCommand(500),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_SPECIMEN_POS)),
                            new SequentialCommandGroup(
                                    new WaitCommand(750),
                                    new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.SPECIMEN)),
                                    new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.SPECIMEN))
                            )
                    )
                    )
            };
        } else if (liftPositionInput == Globals.LIFT_RETRACT_POS) {
            return new Command[]{
                    new InstantCommand(() -> robot.outtakeArmSubsystem.update(Globals.OuttakeArmState.SPECIMEN)),
                    new InstantCommand(() -> robot.depositSubsystem.outtakeSetPosition(Globals.LIFT_RETRACT_POS)),
                    new InstantCommand(() -> robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.SPECIMEN))
            };
        } else {
            return null;
        }
    }
}
