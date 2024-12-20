package org.firstinspires.ftc.teamcode.opmode.autonomous.red;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SecondarySpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SlowerSpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
public class RedHPSide4Specimen extends OpMode {
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;
    Globals.ExtendoFailState extendoFailState;
    Globals.OuttakeClawState outtakeClawState;
    Globals.OuttakeArmState outtakeArmState;
    Globals.FourBarState fourBarState;
    Globals.IntakeClawState intakeClawState;
    Globals.IntakeCoaxialState intakeCoaxialState;
    Globals.IntakeRotationState intakeRotationState;
    Globals.ExtendoState extendoState;
    Globals.OuttakeState outtakeState;

    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_FAR_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_FAR_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_FAR_START_POSE)
                .splineToLinearHeading(new Pose2d(-3, 33.5, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(
                        new Vector2d(-6, 42), Math.toRadians(180),
                        new TranslationalVelConstraint(60)
                )
                .strafeToLinearHeading(
                        new Vector2d(-33, 42), Math.toRadians(180),
                        new TranslationalVelConstraint(60)
                )
                .strafeToLinearHeading(
                        new Vector2d(-33, 20), Math.toRadians(90),
                        new TranslationalVelConstraint(60)
                )
                .strafeToConstantHeading(
                        new Vector2d(-38, 20),
                        new TranslationalVelConstraint(60)
                )
                .strafeToConstantHeading(
                        new Vector2d(-38, 60),
                        new TranslationalVelConstraint(60)
                )
                .strafeToConstantHeading(
                        new Vector2d(-42, 20),
                        new TranslationalVelConstraint(60)
                )
                .strafeToConstantHeading(
                        new Vector2d(-47, 20),
                        new TranslationalVelConstraint(60)
                )
                .strafeToConstantHeading(
                        new Vector2d(-47, 60),
                        new TranslationalVelConstraint(60)
                )
                .strafeToLinearHeading(new Vector2d(-27, 60), Math.toRadians(90))

                .strafeToLinearHeading(
                        new Vector2d(-27, 65), Math.toRadians(90),
                        new TranslationalVelConstraint(12)
                );

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7, 33, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28, 60, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-28, 65, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                );

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-9, 33, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28, 60, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-28, 65, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                );

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-35, 58, Math.toRadians(0)), Math.toRadians(180));

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
        telemetry.addData("Side: ", "Far");
        telemetry.addData("Description: ", "4 Specimen - Slow, Park");
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
                        new WaitCommand(100),
                        new SlowerSpecimenClipCommand(robot),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2A, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new SpecimenIntakeCommand(robot)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SpecimenGrabAndTransferAndLiftCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new ParallelCommandGroup(
                                                new ActionCommand(movement3A, Collections.emptySet())
                                        )
                                )
                        ),
                        new WaitCommand(100),
                        new SecondarySpecimenClipCommand(robot),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new ActionCommand(movement4A, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new SpecimenIntakeCommand(robot)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SpecimenGrabAndTransferAndLiftCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new ParallelCommandGroup(
                                                new ActionCommand(movement5A, Collections.emptySet())
                                        )
                                )
                        ),
                        new WaitCommand(100),
                        new SecondarySpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new ActionCommand(movement6A, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new SpecimenIntakeCommand(robot)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SpecimenGrabAndTransferAndLiftCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new ParallelCommandGroup(
                                                new ActionCommand(movement7A, Collections.emptySet())
                                        )
                                )
                        ),
                        new WaitCommand(100),
                        new SecondarySpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot)
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

        telemetry.addLine("Currently running: 4+0 (4 Specimen)");
        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);
        telemetry.addData("Robot Position: ", robot.pinpointDrive.pose.position);
        telemetry.addData("Extendo State: ", extendoState);
        telemetry.addData("Outtake State: ", outtakeState);
        telemetry.addData("Intake Rotation State: ", intakeRotationState);
        telemetry.addData("Intake Coaxial State: ", intakeCoaxialState);
        telemetry.addData("Intake Claw State: ", intakeClawState);
        telemetry.addData("FourBar State: ", fourBarState);
        telemetry.addData("Outtake Arm State: ", outtakeArmState);
        telemetry.addData("Outtake Claw State: ", outtakeClawState);

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