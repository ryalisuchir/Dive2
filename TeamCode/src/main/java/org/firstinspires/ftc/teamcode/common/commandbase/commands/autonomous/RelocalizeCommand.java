//package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.opencv.core.Point;
//
//import java.util.Vector;
//
//public class RelocalizeCommand extends SequentialCommandGroup {
//    private final RobotHardware robot = new RobotHardware();
//    private double sampleAngle;
//    private Point samplePosition;
//    private Pose2d currentPosition;
//    private Action relocalizeTrajectory;
//
//    public RelocalizeCommand() {
//        super.addCommands(new WaitCommand(50),
//                new InstantCommand(() -> sampleAngle = robot.getSampleAngle()),
//                new InstantCommand(() -> samplePosition = robot.getSamplePosition()),
//                new InstantCommand(
//                        () -> currentPosition = robot.driveSubsystem.getPoseEstimate()),
//                new InstantCommand(
//                        () ->
//                                relocalizeTrajectory =
//                                        robot.driveSubsystem
//                                                .trajectoryActionBuilder(currentPosition)
//                                                .splineTo(new Vector2d(currentPosition.position.x - samplePosition.x, currentPosition.position.y - samplePosition.y), Math.toRadians(0.00))
//                                                .build()),
//                new ActionCommand(relocalizeTrajectory, null),
//                new InstantCommand(() -> robot.intakeRotationSubsystem.updateRotation((sampleAngle % 170) / 180)));
//    }
//}
