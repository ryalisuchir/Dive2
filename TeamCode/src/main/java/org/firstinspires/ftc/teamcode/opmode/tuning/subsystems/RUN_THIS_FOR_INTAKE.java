package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import androidx.core.view.WindowInsetsAnimationCompat;

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
public class RUN_THIS_FOR_INTAKE extends OpMode {
    public static double fourBarPositionGeneral = 0.675;
    public static double coaxialPositionGeneral = 0;
    public static double intakeRotationPosition = 0.52;
    public static double intakeClawPosition = 0.5;

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

        intake4BarRight.setDirection(Servo.Direction.REVERSE);
        intakeCoaxialRight.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        intakeRotation.setPosition(intakeRotationPosition);
        intakeClaw.setPosition(intakeClawPosition);
        intakeCoaxialLeft.setPosition(coaxialPositionGeneral);
        intakeCoaxialRight.setPosition(coaxialPositionGeneral);
        intake4BarLeft.setPosition(fourBarPositionGeneral);
        intake4BarRight.setPosition(fourBarPositionGeneral);
    }
}
