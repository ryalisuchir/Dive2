package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class RUN_THIS_FOR_INTAKE extends OpMode {
    public static double fourBarPositionLeft = 0;
    public static double fourBarPositionRight = 0;
    public static double coaxialPositionLeft = 0;
    public static double coaxialPositionRight = 0;
    public static double intakeRotationPosition = 0;
    public static double intakeClawPosition = 0;

    public Servo intakeRotation, intakeClaw, intakeCoaxialLeft, intakeCoaxialRight, intake4BarLeft, intake4BarRight; //Intake servos

    @Override
    public void init() {
        intakeRotation = hardwareMap.get(Servo.class, "intakeRotation");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeCoaxialLeft = hardwareMap.get(Servo.class, "intakeCoaxial1");
        intakeCoaxialRight = hardwareMap.get(Servo.class, "intakeCoaxial2");
        intake4BarLeft = hardwareMap.get(Servo.class, "intake4Bar1");
        intake4BarRight = hardwareMap.get(Servo.class, "intake4Bar2");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        intakeRotation.setPosition(intakeRotationPosition);
        intakeClaw.setPosition(intakeClawPosition);
        intakeCoaxialLeft.setPosition(coaxialPositionLeft);
        intakeCoaxialRight.setPosition(coaxialPositionRight);
        intake4BarLeft.setPosition(fourBarPositionLeft);
        intake4BarRight.setPosition(fourBarPositionRight);
    }
}
