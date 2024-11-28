package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.SpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.wall.SpecimenGrabAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.wall.TSpecimenTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class SubsystemTest extends CommandOpMode {
    private RobotHardware robot;

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
    }

    @Override
    public void run() {
        robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        0.5 * Math.tan(1.12 * gamepad1.left_stick_y),
                        0.5 * Math.tan(1.12 * gamepad1.left_stick_x)
                ),
                -0.5 * Math.tan(1.12 * gamepad1.right_stick_x)
        ));

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);
        robot.depositSubsystem.outtakeSlidesLoop(Globals.LIFT_P_SLOW);

        if (gamepad1.ps) {
            schedule (
                    new AllSystemInitializeCommand(robot)
            );
        }

        if (gamepad1.cross) {
            schedule(
                    new CloseAndTransferCommand(robot)
            );
        }

        if (gamepad1.circle) {
            schedule(
                    new IntakeCommand(robot, 0.5, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        if (gamepad1.square) {
            schedule (
                    new SpecimenIntakeCommand(robot)
            );
        }

        if (gamepad1.triangle) {
            schedule (
                    new SpecimenGrabAndTransferCommand(robot)
            );
        }

        if (gamepad1.dpad_up) {
            schedule(
                    new BucketDropCommand(robot)
            );
        }

        if (gamepad1.dpad_down) {
            schedule(
                    new SpecimenClipCommand(robot)
            );
        }

    }
}
