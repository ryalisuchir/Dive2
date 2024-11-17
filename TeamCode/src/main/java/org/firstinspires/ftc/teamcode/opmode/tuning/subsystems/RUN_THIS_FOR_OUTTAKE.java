package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
@Disabled
public class RUN_THIS_FOR_OUTTAKE extends OpMode {

    public static double armPositionGeneral = 0;
    public static double outtakeRotationPosition = 0;
    public static double outtakeClawPosition = 0;

    public Servo outtakeRotation, leftOuttakeArm, rightOuttakeArm, outtakeClaw;

    @Override
    public void init() {
        outtakeRotation = hardwareMap.get(Servo.class, "outtakeRotation");
        leftOuttakeArm = hardwareMap.get(Servo.class, "leftOuttakeArm");
        rightOuttakeArm = hardwareMap.get(Servo.class, "rightOuttakeArm");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightOuttakeArm.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        outtakeRotation.setPosition(outtakeRotationPosition);
//        leftOuttakeArm.setPosition(armPositionGeneral);
//        rightOuttakeArm.setPosition(armPositionGeneral);
        outtakeClaw.setPosition(outtakeClawPosition);
    }
}
