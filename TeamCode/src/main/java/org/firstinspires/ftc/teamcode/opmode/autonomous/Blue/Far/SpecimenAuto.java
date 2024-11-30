package org.firstinspires.ftc.teamcode.opmode.autonomous.Blue.Far;

import android.util.Log;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.SlideParkCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.wall.SpecimenGrabAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import java.util.Collections;

@Autonomous
public class SpecimenAuto extends OpMode {
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;
    Globals.ExtendoFailState extendoFailState;
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_FAR_START_POSE);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_FAR_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_FAR_START_POSE)
                .splineToLinearHeading(new Pose2d(-6.70, 32, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-6, 42), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-33, 42), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-33, 20), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-38, 20))
                .strafeToConstantHeading(new Vector2d(-38, 60))
                .strafeToConstantHeading(new Vector2d(-38, 20))
                .strafeToConstantHeading(new Vector2d(-45, 20))
                .strafeToConstantHeading(new Vector2d(-45, 60))
                .strafeToConstantHeading(new Vector2d(-45, 20))
                .strafeToConstantHeading(new Vector2d(-57, 20))
                .strafeToConstantHeading(new Vector2d(-57, 60))
                .strafeToLinearHeading(new Vector2d(-30, 60), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-30, 63), Math.toRadians(90));

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(54, 53, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(59.5, 47, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(54, 55, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(52.8, 30, Math.toRadians(180)), Math.toRadians(40));

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(54, 50, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(18, 8, Math.toRadians(180)), Math.toRadians(180));

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
        telemetry.addData("Ready: ", "All subsystems have been initialized!");
        telemetry.addData("Side: ", "Close");
        telemetry.addData("Description: ", "4 Basket, Park");
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
                        new SequentialCommandGroup(
                                new SpecimenClipCommand(robot),
                                new WaitCommand(300),
                                new OuttakeTransferReadyCommand(robot)
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2A, Collections.emptySet()),
                                new SpecimenIntakeCommand(robot)
                        ),
                        new SpecimenGrabAndTransferCommand(robot)
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

        telemetry.addLine("Currently running: 1+3 (4 High Basket)");
        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        telemetry.addData("Deposit Slides Position: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("Extendo Slides Position: ", robot.extendoMotor.getCurrentPosition());

        if (extendoFailState == Globals.ExtendoFailState.FAILED_EXTEND) {
            Log.i("Extendo Failed:", "FAILED_EXTENSION");
        }

        if (extendoFailState == Globals.ExtendoFailState.FAILED_RETRACT) {
            Log.i("Extendo Failed:", "FAILED_RETRACTION");
        }

        loop = time;
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }

}