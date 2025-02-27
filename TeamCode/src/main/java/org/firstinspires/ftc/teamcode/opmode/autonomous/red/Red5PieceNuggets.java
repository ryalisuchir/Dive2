package org.firstinspires.ftc.teamcode.opmode.autonomous.red;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.wall.SpecimenGrabAndTransferAndLiftCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.ActionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous
public class Red5PieceNuggets extends OpMode {
    private final ArrayList<PathChain> paths = new ArrayList<>();
    private RobotHardware robot;

    public void generatePaths() {
        paths.add( //path 0 - preload specimen to bar
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(7.611, 66.034, Point.CARTESIAN),
                                        new Point(40.937, 66.034, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build()
        );

        paths.add( //path 1 - push all 3 samples
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(40.937, 66.034, Point.CARTESIAN),
                                        new Point(14.811, 50.400, Point.CARTESIAN),
                                        new Point(24.480, 26.537, Point.CARTESIAN),
                                        new Point(89.897, 37.646, Point.CARTESIAN),
                                        new Point(82.903, 23.040, Point.CARTESIAN),
                                        new Point(24.891, 25.509, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                        .addPath(
                                new BezierCurve(
                                        new Point(24.891, 25.509, Point.CARTESIAN),
                                        new Point(0.411, 22.011, Point.CARTESIAN),
                                        new Point(42.377, 22.629, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .addPath(
                                new BezierCurve(
                                        new Point(42.377, 22.629, Point.CARTESIAN),
                                        new Point(92.777, 15.223, Point.CARTESIAN),
                                        new Point(23.451, 14.811, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .addPath(
                                new BezierCurve(
                                        new Point(23.451, 14.811, Point.CARTESIAN),
                                        new Point(1.440, 13.371, Point.CARTESIAN),
                                        new Point(43.406, 12.343, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .addPath(
                                new BezierCurve(
                                        new Point(43.406, 12.343, Point.CARTESIAN),
                                        new Point(96.069, 5.760, Point.CARTESIAN),
                                        new Point(9.463, 6.789, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build()
        );

        paths.add( //path 2 - second specimen on the bar
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(9.463, 6.789, Point.CARTESIAN),
                                        new Point(26.126, 65.623, Point.CARTESIAN),
                                        new Point(39.497, 64.183, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                        .build()
        );

        paths.add( //path 3 - get to the hp for a specimen - regular grab
                robot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.497, 64.183, Point.CARTESIAN),
                                new Point(22.834, 63.566, Point.CARTESIAN),
                                new Point(25.920, 28.800, Point.CARTESIAN),
                                new Point(26.949, 32.914, Point.CARTESIAN),
                                new Point(3.086, 32.503, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build()
        );

        paths.add( //path 4 - get to bar for another specimen drop
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(3.086, 32.503, Point.CARTESIAN),
                                        new Point(18.926, 31.474, Point.CARTESIAN),
                                        new Point(27.566, 66.857, Point.CARTESIAN),
                                        new Point(40.937, 65.829, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                        .build()
        );

    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.SPECIMEN_START_POSE_PEDRO, true);

        generatePaths();

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        telemetry.addData("Ready: ", "All subsystems have been initialized!");
        telemetry.addData("Side: ", "Far");
        telemetry.addData("Description: ", "5 Specimen");
        telemetry.addLine("Good luck 👅");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //First Drop:
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(0)).setMaxPower(0.4),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
                                )
                        ),
                        new WaitCommand(100),
                        new SpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(1)),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new SpecimenIntakeCommand(robot)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SpecimenGrabAndTransferAndLiftCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new FollowPathCommand(robot.follower, paths.get(2))
                                )
                        ),
                        new SpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(3)),
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
                                                new FollowPathCommand(robot.follower, paths.get(4))
                                        )
                                )
                        ),
                        new SpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(3)),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new SpecimenIntakeCommand(robot)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SpecimenGrabAndTransferAndLiftCommand(robot),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ParallelCommandGroup(
                                                new FollowPathCommand(robot.follower, paths.get(4))
                                        )
                                )
                        ),
                        new SpecimenClipCommand(robot),
                        new ParallelCommandGroup(
                                new OuttakeTransferReadyCommand(robot),
                                new FollowPathCommand(robot.follower, paths.get(3))
                        )
                )
        );

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.follower.update();
        robot.depositSubsystem.outtakeSlidesLoop();
        robot.extendoSubsystem.extendoSlidesLoop();

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