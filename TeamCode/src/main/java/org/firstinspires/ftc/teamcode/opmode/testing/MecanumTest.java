package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;

import java.util.Collections;

@Autonomous
public class MecanumTest extends OpMode {

    private DriveRobotHardware robot = DriveRobotHardware.getInstance();
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap, new Pose2d(-6.70, 30.38, Math.toRadians(-90)));

        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-6.70, 30.38, Math.toRadians(-90)));
        Action movement1Left = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(-6.70, 30.38, Math.toRadians(-90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-38.22, 27.95, Math.toRadians(0)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-38.22, 18, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-38.22, 18, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-42, 60), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-42, 20), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-50, 17), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-50, 60), Math.toRadians(90))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ActionCommand(movement1Left, Collections.emptySet())
                )
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        loop = time;
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Closed Camera.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}