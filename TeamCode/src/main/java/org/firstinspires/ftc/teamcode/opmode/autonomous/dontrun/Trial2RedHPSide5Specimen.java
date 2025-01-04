package org.firstinspires.ftc.teamcode.opmode.autonomous.dontrun;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.NoClawScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
@Disabled
public class Trial2RedHPSide5Specimen extends OpMode {
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A, movement9A, movement10A, movement11A, movement12A, movement13A, movement14A;
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_FAR_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_FAR_START_POSE);

        TrajectoryActionBuilder movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_FAR_START_POSE)
                .splineToLinearHeading(new Pose2d(-7.00, 35.00, Math.toRadians(-90.00)), Math.toRadians(-90.00));

        TrajectoryActionBuilder movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-25, 48, Math.toRadians(50)), Math.toRadians(50));

        TrajectoryActionBuilder movement3 = movement2.endTrajectory().fresh()
                .turnTo(Math.toRadians(-15));

        TrajectoryActionBuilder movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-42, 47, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder movement5 = movement4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-42, 59, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement6 = movement5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-42, 64, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder movement7 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7, 37, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7, 32.5, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement8 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 60, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-30, 65, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                );

        TrajectoryActionBuilder movement9 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7, 37, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7, 32.5, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement10 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 60, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-30, 65, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                );

        TrajectoryActionBuilder movement11 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7, 37, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7, 32.5, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement12 = movement7.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 60, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(
                        new Pose2d(-30, 65, Math.toRadians(90)), Math.toRadians(90),
                        new TranslationalVelConstraint(15)
                );

        TrajectoryActionBuilder movement13 = movement6.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-7, 37, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-7, 32.5, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder movement14 = movement7.endTrajectory().fresh()
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
        movement9A = movement9.build();
        movement10A = movement10.build();
        movement11A = movement11.build();
        movement12A = movement12.build();
        movement13A = movement13.build();
        movement14A = movement14.build();
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
                        new ParallelCommandGroup(
                                new ActionCommand(movement1A, Collections.emptySet()),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
                                )
                        ),
                        new WaitCommand(100),
                        new SpecimenClipCommand(robot),
                        new ActionCommand(movement2A, Collections.emptySet()),
                        new ScanningCommand(robot, 0.28, Globals.EXTENDO_MAX_EXTENSION * 0.9),
                        new WaitCommand(100),
                        new IntakePeckerCommand(robot),
                        new ParallelCommandGroup(
                                new ActionCommand(movement3A, Collections.emptySet()),
                                new NoClawScanningCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION)
                        ),
                        new InstantCommand(() -> robot.intakeClawSubsystem.update(Globals.IntakeClawState.OPEN))
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

        telemetry.addLine("Currently running: 4+0 (4 Specimen)");
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