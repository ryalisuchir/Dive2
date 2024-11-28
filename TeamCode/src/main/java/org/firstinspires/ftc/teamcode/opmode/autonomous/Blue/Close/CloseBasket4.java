package org.firstinspires.ftc.teamcode.opmode.autonomous.Blue.Close;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Collections;

@Autonomous
public class CloseBasket4 extends OpMode {
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_CLOSE_START_POSE_NEW);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_CLOSE_START_POSE_NEW);
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

        Action movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_CLOSE_START_POSE_NEW)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(54, 54, Math.toRadians(45.00)), Math.toRadians(90.00))
                .build();

        Action movement2 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(54, 54, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(45, 47, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action movement3 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(45, 47, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(54, 54, Math.toRadians(45)), Math.toRadians(45))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(movement1, Collections.emptySet()),
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                        ),
                        new WaitCommand(350),
                        new BucketDropCommand(robot),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new ActionCommand(movement2, Collections.emptySet()),
                                new OuttakeTransferReadyCommand(robot)
                        ),
                        new WaitCommand(150),
                        new IntakeCommand(robot, 0.5, 900),
                        new WaitCommand(150),
                        new CloseAndTransferCommand(robot),
                        new WaitCommand(150),
                        new ParallelCommandGroup(
                                new ActionCommand(movement3, Collections.emptySet()),
                                new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                        )
                )


//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new ActionCommand(movement1, Collections.emptySet()),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1200),
//                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
//                                )
//                        ),
//                        new WaitCommand(500),
//                        new BucketDropCommand(robot),
//                        new WaitCommand(500),
//                        new ParallelCommandGroup(
//                                new IntakeCommand(robot, Globals.INTAKE_ROTATION_AUTO_1, 1000),
//                                new ActionCommand(movement2, Collections.emptySet()),
//                                new OuttakeTransferReadyCommand(robot)
//                        ),
//                        new WaitCommand(350),
//                        new CloseAndTransferCommand(robot),
////                        new ActionCommand(movement3, Collections.emptySet()),
//                    new OuttakeCommand(robot, Globals.LIFT_HIGH_POS),
//                    new WaitCommand(100),
//                    new BucketDropCommand(robot)
//                )
        );

    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.depositSubsystem.outtakeSlidesLoop(Globals.LIFT_P_SLOW);
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

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