package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

import java.util.Collections;

@Photon
@Autonomous
public class MecanumTest extends OpMode {

    private DriveRobotHardware robot = DriveRobotHardware.getInstance();
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap, Globals.BLUE_CLOSE_START_POSE);

        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_CLOSE_START_POSE);
        Action movement1Left = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_CLOSE_START_POSE)
                .splineToConstantHeading(new Vector2d(6.36, 32.65), Math.toRadians(270.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(56, 51.28, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(53, 43.10, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(56, 51.28, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(60.16, 42.05, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(61.20, 50.58, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(58, 42.05, Math.toRadians(140)), Math.toRadians(140))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(60.68, 52.15, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(23.77, 0.96, Math.toRadians(180.00)), Math.toRadians(180.00),
                        new AngularVelConstraint(80)
                )
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