package org.firstinspires.ftc.teamcode.opmode.autonomous.red;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HangUpCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.SlideParkCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.RegularTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.SlowIntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
@Disabled
public class RedBucketSide1Specimen3Sample extends OpMode {
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A;
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_CLOSE_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_CLOSE_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_CLOSE_START_POSE)
                .splineToLinearHeading(new Pose2d(6, 34, Math.toRadians(270.00)), Math.toRadians(270));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.9, 54.5, Math.toRadians(90)), Math.toRadians(-90.00));

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(65.6, 56, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(66, 51, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(64.2, 57, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh()   //edit this one goddamn it suchir
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(53, 47.2, Math.toRadians(140)), Math.toRadians(40));

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(64, 59, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35.96, 10.00, Math.toRadians(-180.00)), Math.toRadians(-180.00))
                .splineToLinearHeading(
                        new Pose2d(21.00, 10.00, Math.toRadians(-180.00)), Math.toRadians(-180.00),
                        new TranslationalVelConstraint(15)
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
                new ParallelCommandGroup(
                        new HangUpCommand(robot.hangSubsystem, 1, 1230),
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
                                //First Intake:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement2A, Collections.emptySet()),
                                        new OuttakeTransferReadyCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2400),
                                                new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.85)
                                        )
                                ),
                                new WaitCommand(150),
                                new SlowIntakePeckerCommand(robot),
                                new ParallelCommandGroup(
                                        new RegularTransferCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(130),
                                                new ActionCommand(movement3A, Collections.emptySet())
                                        )
                                ),
                                //Second Drop:
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                                new ParallelCommandGroup(
                                        new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.65),
                                        new BucketDropCommand(robot)
                                ),
                                //Second Intake:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement4A, Collections.emptySet()),
                                        new OuttakeTransferReadyCommand(robot)
                                ),
                                new WaitCommand(150),
                                new SlowIntakePeckerCommand(robot),
                                new ParallelCommandGroup(
                                        new RegularTransferCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new ActionCommand(movement5A, Collections.emptySet())
                                        )
                                ),
                                //Third Drop:
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                                new BucketDropCommand(robot),
                                //Fourth Intake:
                                new ParallelCommandGroup(
                                        new ActionCommand(movement6A, Collections.emptySet()),
                                        new IntakeCommand(robot, 0.4, Globals.EXTENDO_MAX_EXTENSION),
                                        new OuttakeTransferReadyCommand(robot)
                                ),
                                new WaitCommand(150),
                                new SlowIntakePeckerCommand(robot),
                                new ParallelCommandGroup(
                                        new RegularTransferCommand(robot),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new ActionCommand(movement7A, Collections.emptySet())
                                        )
                                ),
                                //Fourth Drop:
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
                                new BucketDropCommand(robot),
                                //Park
                                new ParallelCommandGroup(
                                        new SlideParkCommand(robot),
                                        new ActionCommand(movement8A, Collections.emptySet())
                                )
                        )
                )
        );

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.depositSubsystem.outtakeSlidesLoop();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();

        telemetry.addLine("Currently running: 1+3 (1 Specimen 3 High Basket)");
        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);
        telemetry.addData("Robot Position: ", robot.pinpointDrive.pose.position);
        telemetry.addData("Extendo State: ", Globals.extendoState);
        telemetry.addData("Outtake State: ", Globals.outtakeState);
        telemetry.addData("Intake Rotation State: ", Globals.intakeRotationState);
        telemetry.addData("Intake Coaxial State: ", Globals.intakeCoaxialState);
        telemetry.addData("Intake Claw State: ", Globals.intakeClawState);
        telemetry.addData("FourBar State: ", Globals.fourBarState);
        telemetry.addData("Outtake Arm State: ", Globals.outtakeArmState);
        telemetry.addData("Outtake Claw State: ", Globals.outtakeClawState);

        if (Globals.extendoFailState == Globals.ExtendoFailState.FAILED_EXTEND) {
            Log.i("Extendo Failed:", "FAILED_EXTENSION");
        }

        if (Globals.extendoFailState == Globals.ExtendoFailState.FAILED_RETRACT) {
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