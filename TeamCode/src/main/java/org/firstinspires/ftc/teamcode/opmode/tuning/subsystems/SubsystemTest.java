package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.HighBucketOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ClawTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.OuttakeTransferReadyCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class SubsystemTest extends CommandOpMode {
    private RobotHardware robot;
    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop();
        robot.depositSubsystem.outtakeSlidesLoop();

        robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -0.5 * Math.tan(1.12 * gamepad1.left_stick_y),
                        -0.5 * Math.tan(1.12 * gamepad1.left_stick_x)
                ),
                -0.5 * Math.tan(1.12 * gamepad1.right_stick_x)
        ));

        boolean triangle = gamepad1.triangle;
        if (triangle) {
            schedule (
                    new AllSystemInitializeCommand(robot)
            );
        }

        boolean circle = gamepad1.circle;
        if (circle) {
            schedule(
                    new IntakeCommand(robot, 0.5, 500)
            );
        }

        boolean square = gamepad1.square;
        if (square) {
            schedule(
                    new ScanningCommand(robot, 0.5, 500)
            );
        }

        boolean x = gamepad1.cross;
        if (x) {
            schedule(
                    new CloseAndTransferCommand(robot)
            );
        }

        boolean up = gamepad1.dpad_up;
        if (up) {
            schedule (
                    new HighBucketOuttakeCommand(robot)
            );
        }
        boolean down = gamepad1.dpad_down;
        if (down) {
            schedule (
                    new BucketDropCommand(robot)
            );
        }

        boolean left = gamepad1.dpad_right;
        if (left) {
            schedule (
                    new OuttakeTransferReadyCommand(robot)
            );
        }


    }
}
