package org.firstinspires.ftc.teamcode.opmode.tuning;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;


@Autonomous
@Disabled
public class EncoderPositions extends OpMode {

    AnalogInput analogInput;
    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);

        telemetry.addData("Ready: ", "Initialized robot.");
        telemetry.update();

    }

    @Override
    public void loop() {
        //Motors:
        telemetry.addLine("Motor Encoders:");
        telemetry.addData("leftLift: ", robot.leftLift.getCurrentPosition());
        telemetry.addData("rightLift: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("extendoMotor: ", robot.extendoMotor.getCurrentPosition());

        telemetry.addLine("IMU:");
        telemetry.addData("Pitch (Degrees): ", robot.pinpointDrive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Degrees): ", robot.pinpointDrive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));


        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Closed OpMode.");
    }
}