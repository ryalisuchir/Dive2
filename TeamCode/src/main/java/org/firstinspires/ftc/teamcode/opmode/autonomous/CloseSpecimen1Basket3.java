package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.SlideParkCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.slow.SlowCloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
public class CloseSpecimen1Basket3 extends OpMode {
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;
    Globals.ExtendoFailState extendoFailState;
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_CLOSE_START_POSE);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_CLOSE_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_CLOSE_START_POSE)
                .splineToLinearHeading(new Pose2d(7.23, 31.78, Math.toRadians(270.00)), Math.toRadians(270));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(58, 41, Math.toRadians(90)), Math.toRadians(-90.00));

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(65, 54.5, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(70, 48, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(65, 54.5, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(63, 32.6, Math.toRadians(180)), Math.toRadians(40));

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(65, 54.5, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35.96, 10.00, Math.toRadians(-180.00)), Math.toRadians(-180.00))
                .splineToLinearHeading(
                        new Pose2d(21.00, 10.00, Math.toRadians(-180.00)), Math.toRadians(-180.00),
                        new TranslationalVelConstraint((double) 15)
                );

        movement1A = movement1.build();
        movement2A = movement2.build();
        movement3A = movement3.build();
        movement4A = movement4.build();
        movement5A = movement5.build();
        movement6A = movement6.build();
        movement7A = movement7.build();
        movement8A = movement8.build();
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        telemetry.addData("Ready: ", "All subsystems have been initialized!");
        telemetry.addData("Side: ", "Close");
        telemetry.addData("Description: ", "1 Specimen, 3 Basket, Park");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //First Drop:
                        new ParallelCommandGroup(
                                new ActionCommand(movement1A, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
                                )
                        ),
                        new SpecimenClipCommand(robot),
                        new WaitCommand(150),
                        //First Intake:
                        new ParallelCommandGroup(
                                new ActionCommand(movement2A, Collections.emptySet()),
                                new OuttakeTransferReadyCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(2200),
                                        new IntakeCommand(robot, 0.39, 400)
                                )
                        ),
                        new WaitCommand(900),
                        new ParallelCommandGroup(
                                new CloseAndTransferCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new ActionCommand(movement3A, Collections.emptySet())
                                )
                        ),
                        //Second Drop:
                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                        new WaitCommand(150),
                        new ParallelCommandGroup(
                                new IntakeCommand(robot, 0.39, 950),
                                new BucketDropCommand(robot)
                        ),
                        new WaitCommand(150),
                        //Second Intake:
                        new ParallelCommandGroup(
                                new ActionCommand(movement4A, Collections.emptySet()),
                                new OuttakeTransferReadyCommand(robot)
                        ),
                        new WaitCommand(800),
                        new ParallelCommandGroup(
                                new CloseAndTransferCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new ActionCommand(movement5A, Collections.emptySet())
                                )
                        ),
                        //Third Drop:
                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                        new WaitCommand(150),
                        new ParallelCommandGroup(
                                new IntakeCommand(robot, 0.68, 400),
                                new BucketDropCommand(robot)
                        ),
                        new WaitCommand(150),
                        //Fourth Intake:
                        new ParallelCommandGroup(
                                new ActionCommand(movement6A, Collections.emptySet()),
                                new OuttakeTransferReadyCommand(robot)
                        ),
                        new WaitCommand(800),
                        new ParallelCommandGroup(
                                new SlowCloseAndTransferCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new ActionCommand(movement7A, Collections.emptySet())
                                )
                        ),
                        //Fourth Drop:
                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                        new BucketDropCommand(robot),
                        new WaitCommand(150),
                        //Park
                        new ParallelCommandGroup(
                                new SlideParkCommand(robot),
                                new ActionCommand(movement8A, Collections.emptySet())
                        )
                )
        );

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.depositSubsystem.outtakeSlidesLoop(Globals.LIFT_P_SLOW);
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);

        telemetry.addLine("Currently running: 1+3 (1 Specimen 3 High Basket)");
        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        if (extendoFailState == Globals.ExtendoFailState.FAILED_EXTEND) {
            Log.i("Extendo Failed:", "FAILED_EXTENSION");
        }

        if (extendoFailState == Globals.ExtendoFailState.FAILED_RETRACT) {
            Log.i("Extendo Failed:", "FAILED_RETRACTION");
        }

        loop = time;
        telemetry.update();
        robot.clearCache();
    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }

}