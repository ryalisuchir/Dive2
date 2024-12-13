package org.firstinspires.ftc.teamcode.opmode.tuning;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;


@Autonomous
public class ResetEncoders extends OpMode {

    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized robot.");
        telemetry.update();

    }

    @Override
    public void loop() {
        telemetry.addLine("Cleared encoder readings.");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Closed OpMode.");
    }
}