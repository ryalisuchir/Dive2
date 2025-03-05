package org.firstinspires.ftc.teamcode.opmode.autonomous.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.ActionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.pathing.roadrunner.MecanumDrive;

import java.util.Collections;

@Autonomous(preselectTeleOp = "Duo")
public class Blue4PieceNuggets extends OpMode {
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A;
    private RobotHardware robot;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_FAR_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_FAR_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.whatTheSigma(Globals.BLUE_FAR_START_POSE)
                .splineToLinearHeading(new Pose2d(-7, 34, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(
                        new Vector2d(-6, 42), Math.toRadians(180),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .strafeToLinearHeading(
                        new Vector2d(-33, 42), Math.toRadians(180),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .strafeToLinearHeading(
                        new Vector2d(-40, 20), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-40, 60)
                )
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-43, 30), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-41, 50), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .strafeToConstantHeading(
                        new Vector2d(-38, 20),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .strafeToConstantHeading(
                        new Vector2d(-49, 20),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .strafeToConstantHeading(
                        new Vector2d(-49, 50),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .strafeToLinearHeading(new Vector2d(-28, 58), Math.toRadians(90))

                .strafeToLinearHeading(
                        new Vector2d(-28, 64), Math.toRadians(90),
                        new TranslationalVelConstraint(6)
                );

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7, 34, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-27, 58, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-27, 64, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(6)
                );

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7.5, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7.5, 34, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28, 58, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-28, 64, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(6)
                );

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-15, 58, Math.toRadians(0)), Math.toRadians(180),
                        null,
                        new ProfileAccelConstraint(-60, 85)
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
        telemetry.addData("Side: ", "Far");
        telemetry.addData("Description: ", "4 Specimen - Lightning McQueen Ahh Speed, Park");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
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
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new ActionCommand(movement3A, Collections.emptySet())
                                        )
                                )
                        ),
                        new WaitCommand(100),
                        new SpecimenClipCommand(robot),
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
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new ActionCommand(movement5A, Collections.emptySet())
                                        )
                                )
                        ),
                        new SpecimenClipCommand(robot),
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
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new ActionCommand(movement7A, Collections.emptySet())
                                        )
                                )
                        ),
                        new SpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot),
                                new ActionCommand(movement8A, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new WaitCommand(700),
                                        new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION)
                                )
                        )
                )
        );

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.depositSubsystem.outtakeSlidesLoop(0.0001);
        robot.extendoSubsystem.extendoSlidesLoop();

        robot.clearCache();
    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }

}