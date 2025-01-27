package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.HangUpCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Autonomous
@Config
@Disabled
public class HangTest extends OpMode {
    private RobotHardware robot;
    AnalogInput rightInput, leftInput;
    public static double target = 120;
    public static double p = 0.02;

    boolean isDoneGoingOut = false;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.BLUE_SIDEWAYS_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized subsystems.");
        telemetry.update();

        CommandScheduler.getInstance().schedule(new AllSystemInitializeCommand(robot));
        robot.driveSubsystem.setPoseEstimate(Globals.BLUE_SIDEWAYS_START_POSE);

        rightInput = hardwareMap.get(AnalogInput.class, "hangRightInput");
        leftInput = hardwareMap.get(AnalogInput.class, "hangLeftInput");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        robot.clearCache();
        telemetry.addData("Ready: ", "All subsystems have been initialized!");
        telemetry.addData("Side: ", "N/A");
        telemetry.addData("Description: ", "Hang Hook Test");
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(new HangUpCommand(robot.hangSubsystem, -1, 2200).whenFinished(() -> {isDoneGoingOut = true;}));
    }

    @Override
    public void loop() {

        if (isDoneGoingOut) {
            double error = target - rightInput.getVoltage() / 3.3 * 360;
            double error2 = target - leftInput.getVoltage() / 3.3 * 360;

            robot.rightHang.setPower(error * p);
//            robot.leftHang.setPower(error2 * p);

            if (error < 5 && error2 < 5) {
                isDoneGoingOut = false; //exit the loop
            }

        }

        CommandScheduler.getInstance().run();
        robot.clearCache();
    }

    @Override
    public void stop() {
        telemetry.addLine("Ended OpMode.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}