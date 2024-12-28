package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

public class RunLaterCommand extends InstantCommand {
    public RunLaterCommand(RobotHardware robot, Pose2d currPos, double x, double y) {
        super(() -> {
            Action action = robot.driveSubsystem.trajectoryActionBuilder(currPos)
                    .strafeToConstantHeading(new Vector2d(x, y))
                    .build();

            CommandScheduler.getInstance().schedule(new ActionCommand(action, Collections.emptySet()));
        });
    }
}