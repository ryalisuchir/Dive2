
/*
package org.firstinspires.ftc.teamcode.opmode.autonomous.Blue.Close;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
@Photon
public class CloseBasket4 extends OpMode {
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_CLOSE_START_POSE);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        robot.intakeClawSubsystem.update(Globals.IntakeClawState.CLOSED);
        robot.intakeCoaxialSubsystem.update(Globals.IntakeCoaxialState.REST);
        robot.intake4BarSubsystem.update(Globals.FourBarState.RESTING);
        robot.intakeRotationSubsystem.update(Globals.IntakeRotationState.REST);
//        robot.outtakeRotationSubsystem.update(Globals.OuttakeRotationState.TRANSFER);
//        robot.outtakeClawSubsystem.update(Globals.OuttakeClawState.CLOSED);
//        robot.outtakeArmSubsystem.outtakeArmTransfer();

        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_CLOSE_START_POSE);


    }

    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        Action movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_CLOSE_START_POSE)
                .splineToConstantHeading(new Vector2d(6.36, 32.65), Math.toRadians(270.00))
                .build();

        Action movement2 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(6.36, 32.65, Math.toRadians(270.00)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(56, 51.28, Math.toRadians(60)), Math.toRadians(60))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(movement1, Collections.emptySet())
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2, Collections.emptySet()),
                                new IntakeCommand1(robot, 750)
                        )
                )
        );

    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
//        robot.depositSubsystem.outtakeSlidesLoop();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();

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
*/