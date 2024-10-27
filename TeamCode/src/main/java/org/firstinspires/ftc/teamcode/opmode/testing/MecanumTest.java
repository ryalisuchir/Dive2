package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.DriveRobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

import java.util.Collections;

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

        telemetry.addData("Not Ready: ", "Not able to proceed to camera detection... Restart robot now.");
        telemetry.update();

    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_CLOSE_START_POSE);
        Action movement1Left = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_CLOSE_START_POSE)
                .splineToConstantHeading(new Vector2d(6.36, 32.65), Math.toRadians(270.00))
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