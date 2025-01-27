package org.firstinspires.ftc.teamcode.opmode.autonomous.dontrun;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
@Disabled
public class Blue5SpecimenPathing extends OpMode {
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A;
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_FAR_START_POSE_REVERSED, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_FAR_START_POSE_REVERSED);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_FAR_START_POSE_REVERSED)
                .setReversed(true)
                .splineTo(
                        new Vector2d(-44, 12), Math.toRadians(270.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .setReversed(false)
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-52, 30), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-52, 55), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                //pushing second one:
                .splineToConstantHeading(
                        new Vector2d(-52, 15), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-65, 30), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-65, 60), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                //pushing third one:
                .strafeToConstantHeading(new Vector2d(-65, 60), null, new ProfileAccelConstraint(-60, 85));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-65, 64), null, new ProfileAccelConstraint(-60, 85));

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .setReversed(true)
                //first dropoff
                .splineToLinearHeading(new Pose2d(-14, 37, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-14, 30, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 60, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-30, 68, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                );

        movement1A = movement1.build();
        movement2A = movement2.build();
        movement3A = movement3.build();
        movement4A = movement4.build();
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        telemetry.addData("Ready: ", "All subsystems have been initialized!");
        telemetry.addData("Side: ", "Far");
        telemetry.addData("Description: ", "5 Specimen");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ActionCommand(movement1A, Collections.emptySet()),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2A, Collections.emptySet()),
                                new SpecimenIntakeCommand(robot)
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

        telemetry.addLine("Currently running: 5+0 (5 Specimen)");
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