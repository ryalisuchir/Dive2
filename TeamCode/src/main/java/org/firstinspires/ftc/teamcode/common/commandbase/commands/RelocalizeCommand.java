package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;

public class RelocalizeCommand extends SequentialCommandGroup {
    private final RobotHardware robot = RobotHardware.getInstance();
    private double sampleAngle;
    private Point samplePosition;
    private Pose2d currentPosition;
    private TrajectorySequence relocalizeTrajectory;
    double lastDetectedBlue = 0;
    Point lastDetectedPoint = new Point(0, 0);

    public RelocalizeCommand() {
        super.addCommands(new WaitCommand(50),
                new InstantCommand(() -> sampleAngle = robot.getSampleAngle()),
                new InstantCommand(() -> samplePosition = robot.getSamplePosition()),
        new InstantCommand(
                () -> currentPosition = robot.driveSubsystem.getPoseEstimate()),
                new InstantCommand(
                        () ->
                                relocalizeTrajectory =
                                robot.driveSubsystem
                                        .trajectorySequenceBuilder(currentPosition)
                                        .lineToLinearHeading(new Pose2d(
                                                currentPosition.getX() - samplePosition.x,
                                                currentPosition.getY() - samplePosition.y,
                                                Math.toRadians(0.00)))
                                        .build()),
                new DriveCommand(robot.driveSubsystem, relocalizeTrajectory),
                new InstantCommand(() -> robot.intakeRotationSubsystem.updateRotation((sampleAngle % 170) / 180)));
    }
}
