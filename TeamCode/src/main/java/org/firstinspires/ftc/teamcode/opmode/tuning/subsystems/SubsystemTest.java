package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.intake.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.outtake.BucketDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.transfer.ground.CloseAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.teleop.transfer.wall.TSpecimenTransferCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp
public class SubsystemTest extends CommandOpMode {
    private RobotHardware robot;
    private boolean depositManualControl;
    private boolean extendoManualControl;

    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE);
    }

    @Override
    public void run() {

        //Loop:
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.updatePoseEstimate();
        robot.extendoSubsystem.currentLoop();
        robot.extendoSubsystem.extendoSlidesLoop(Globals.EXTENDO_P_SLOW);

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
                    new TSpecimenTransferCommand(robot)
            );
        }

        if (gamepad1.dpad_up) {
            schedule (
                    new AllSystemInitializeCommand(robot)
            );
        }

        if (gamepad1.dpad_down) {
            schedule(
                    new BucketDropCommand(robot)
            );
        }

    }
}
