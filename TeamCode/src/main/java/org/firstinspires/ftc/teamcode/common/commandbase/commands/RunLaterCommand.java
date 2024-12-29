package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive.DriveSubsystem;

import java.util.Collections;

public class RunLaterCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    Action action;
    private double xPos, yPos;
    private Pose2d currPos;

    public RunLaterCommand(DriveSubsystem driveSubsystemInput, Pose2d currPosInput, double x, double y) {
        this.driveSubsystem = driveSubsystemInput;
        this.xPos = x;
        this.yPos = y;
        this.currPos = currPosInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        action = driveSubsystem.trajectoryActionBuilder(currPos)
                .strafeToConstantHeading(new Vector2d(xPos, yPos))
                .build();
        CommandScheduler.getInstance().schedule(new ActionCommand(action, Collections.emptySet()));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveSubsystem.getPoseEstimate().position.x - xPos) < 0.2 &&
                Math.abs(driveSubsystem.getPoseEstimate().position.y - yPos) < 0.2;
    }


}
