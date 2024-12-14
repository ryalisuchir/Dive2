package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.outtake.specimen.SecondarySpecimenClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop.CLCloseAndTransfer;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop.CLRetractedCloseAndTransfer;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.ground.teleop.IntakePeckerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.transfer.wall.SpecimenGrabAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class SubsystemTest extends CommandOpMode {
    private RobotHardware robot;
    private boolean isCloseAndTransfer = true; // Track toggle state

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);
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

        telemetry.addData("Extendo:", robot.extendoMotor.getCurrentPosition());
        telemetry.update();

        if (gamepad1.ps) {
            schedule(
                    new AllSystemInitializeCommand(robot)
            );
        }

        if (gamepad1.dpad_left) {
            if (isCloseAndTransfer) {
                new IntakePeckerCommand(robot).schedule();
            } else {
                new InstantCommand(() -> robot.intakeClawSubsystem.intakeClawOpen()).schedule();
            }
            isCloseAndTransfer = !isCloseAndTransfer;
        }
        ;

        if (gamepad1.cross) {
            if (robot.extendoMotor.getCurrentPosition() < 200) {
                schedule(
                        new CLRetractedCloseAndTransfer(robot)
                );
            } else {
                schedule(
                        new CLCloseAndTransfer(robot)
                );
            }
        }

        if (gamepad1.circle) {
            schedule(
                    new IntakeCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        if (gamepad1.left_trigger > 0) {
            schedule(
                    new IntakeCommand(robot, 0.55, Globals.EXTENDO_MAX_EXTENSION)
            );
        }

        if (gamepad1.square) {
            schedule(
                    new SpecimenIntakeCommand(robot)
            );
        }

        if (gamepad1.triangle) {
            schedule(
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
                    new SecondarySpecimenClipCommand(robot)
            );
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            schedule(
                    new OuttakeCommand(robot, Globals.LIFT_SPECIMEN_POS)
            );
        }

    }
}
