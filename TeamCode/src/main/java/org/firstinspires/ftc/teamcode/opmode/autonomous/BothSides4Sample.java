package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.SetIntakeDownCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.RetractedTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.utility.ActionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.ZoneLookupTable;

import java.util.Collections;

@Autonomous(preselectTeleOp = "Duo")
@Disabled
public class BothSides4Sample extends OpMode { //may veer bless us
    Action movement1A, movement2A, movement3A, movement4A, movement5A, movement6A, movement7A, movement8A, movement9A, movement9B, movement10A;
    TrajectoryActionBuilder movement1, movement2, movement3, movement4, movement5, movement6, movement7, movement9, movement92, movement10;
    //Vision Initialization:
    //Human Input for Vision:
    ZoneLookupTable lookupTable;
    private RobotHardware robot;


    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        lookupTable = new ZoneLookupTable();

        robot = new RobotHardware(hardwareMap, Globals.BLUE_SIDEWAYS_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_SIDEWAYS_START_POSE);

        movement1 = robot.driveSubsystem.trajectoryActionBuilder(Globals.BLUE_SIDEWAYS_START_POSE)
                .splineToLinearHeading(
                        new Pose2d(61, 59, Math.toRadians(45)), Math.toRadians(45),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                );

        movement2 = movement1.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(
                        new Pose2d(54, 46, Math.toRadians(90)), (Math.toRadians(0)),
                        null,
                        new ProfileAccelConstraint(-35, 35)
                );

        movement3 = movement2.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(61, 57, Math.toRadians(45)), Math.toRadians(45));

        movement4 = movement3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(
                                63.6, 43, Math.toRadians(90)), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-35, 35)
                );

        movement5 = movement4.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(61, 56, Math.toRadians(45)), Math.toRadians(45));

        movement6 = movement5.endTrajectory().fresh() //3rd sample grab
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(58, 46, Math.toRadians(145)), Math.toRadians(40),
                        new AngularVelConstraint(Math.PI * 0.8)
                );

        movement7 = movement6.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(59, 56, Math.toRadians(45)), Math.toRadians(45));

        movement9 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(27, 7, Math.toRadians(0)))
                .setReversed(false)
                .splineTo(
                        new Vector2d(58, 53), Math.toRadians(45.00)
                )
                .splineTo(
                        new Vector2d(61, 55), Math.toRadians(45.00)
                )
        ;

        movement92 = robot.driveSubsystem.trajectoryActionBuilder(new Pose2d(27, 3, Math.toRadians(0)))
                .setReversed(false)
                .splineTo(
                        new Vector2d(59, 58), Math.toRadians(45.00),
                        null,
                        new ProfileAccelConstraint(-30, 85)
                )
                .splineTo(
                        new Vector2d(61, 60), Math.toRadians(45.00),
                        null,
                        new ProfileAccelConstraint(-30, 85)
                )
        ;

        movement10 = movement92.endTrajectory().fresh()
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(56, 54), null, new ProfileAccelConstraint(-85, 85));

        movement1A = movement1.build();
        movement2A = movement2.build();
        movement3A = movement3.build();
        movement4A = movement4.build();
        movement5A = movement5.build();
        movement6A = movement6.build();
        movement7A = movement7.build();
        movement9A = movement9.build();
        movement9B = movement92.build();
        movement10A = movement10.build();
    }

    @Override
    public void init_loop() {
        robot.clearCache();
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
                                        new WaitCommand(100),
                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                                ),
                                new SetIntakeDownCommand(robot)
                        ),
                        //First Intake:
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(0),
                                        new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.28)
                                ),
                                new SequentialCommandGroup(
                                        new BucketDropCommand(robot),
                                        new OuttakeTransferReadyCommand(robot)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(50),
                                        new ActionCommand(movement2A, Collections.emptySet())
                                )
                        ),
                        new WaitCommand(250),
                        new IntakePeckerCommand(robot),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new RetractedTransferCommand(robot),
                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(130),
                                        new ActionCommand(movement3A, Collections.emptySet())
                                )
                        ),
                        //Second Drop:
                        //Second Intake:
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ActionCommand(movement4A, Collections.emptySet())
                                ),
                                new SequentialCommandGroup(
                                        new BucketDropCommand(robot),
                                        new OuttakeTransferReadyCommand(robot)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(0),
                                        new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION * 0.2)
                                )
                        ),
                        new WaitCommand(150),
                        new IntakePeckerCommand(robot),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new RetractedTransferCommand(robot),
                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new ActionCommand(movement5A, Collections.emptySet())
                                )
                        ),
                        //Third Drop:
                        //Fourth Intake:
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ActionCommand(movement6A, Collections.emptySet())
                                ),
                                new SequentialCommandGroup(
                                        new BucketDropCommand(robot),
                                        new OuttakeTransferReadyCommand(robot)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(0),
                                        new IntakeCommand(robot, 0.75, Globals.EXTENDO_MAX_EXTENSION * 0.6)
                                )
                        ),
                        new WaitCommand(150),
                        new IntakePeckerCommand(robot),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new RetractedTransferCommand(robot),
                                        new OuttakeCommand(robot, Globals.LIFT_HIGH_POS)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new ActionCommand(movement7A, Collections.emptySet())
                                )
                        ),
                        //Fourth Drop:
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ActionCommand(movement8A, Collections.emptySet())
                                ),
                                new SequentialCommandGroup(
                                        new BucketDropCommand(robot),
                                        new OuttakeTransferReadyCommand(robot)
                                )
                        )
                )
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.depositSubsystem.outtakeSlidesLoop(0.0002);
        robot.extendoSubsystem.extendoSlidesLoop(0.013,0,0.00025,0);

        robot.clearCache();
    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}