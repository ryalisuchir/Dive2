package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;

    private final org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence trajectory;

    public DriveCommand(DriveSubsystem drive, org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}