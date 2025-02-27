package org.firstinspires.ftc.teamcode.opmode.tuning.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class GearResetter extends OpMode {
    public static double fourBarPositionGeneral = 0.675;
    public static double coaxialPositionGeneral = 0;
    public static double intakeRotationPosition = 0.52;
    public static double intakeClawPosition = 0.5;

    public static double armPositionGeneral = 0.76;
    public static double outtakeClawPosition = 0.4;

    public static double hangPower = 0;
    public Servo intakeRotation, intakeClaw, intakeCoaxialLeft, intakeCoaxialRight, intake4BarLeft, intake4BarRight; //Intake servos
    public Servo leftOuttakeArm, rightOuttakeArm, outtakeClaw;
    public CRServo leftHang, rightHang;
    AnalogInput analogInput;

    @Override
    public void init() {
        intakeRotation = hardwareMap.get(Servo.class, "intakeRotation");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeCoaxialLeft = hardwareMap.get(Servo.class, "intakeCoaxial1");
        intakeCoaxialRight = hardwareMap.get(Servo.class, "intakeCoaxial2");
        intake4BarLeft = hardwareMap.get(Servo.class, "intake4Bar1");
        intake4BarRight = hardwareMap.get(Servo.class, "intake4Bar2");

        analogInput = hardwareMap.get(AnalogInput.class, "hangLeftInput");

        leftOuttakeArm = hardwareMap.get(Servo.class, "leftOuttakeArm");
        rightOuttakeArm = hardwareMap.get(Servo.class, "rightOuttakeArm");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        leftHang = hardwareMap.get(CRServo.class, "leftHang");
        rightHang = hardwareMap.get(CRServo.class, "rightHang");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake4BarRight.setDirection(Servo.Direction.REVERSE);
        intakeCoaxialRight.setDirection(Servo.Direction.REVERSE);
        rightOuttakeArm.setDirection(Servo.Direction.REVERSE);
        rightHang.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        intakeRotation.setPosition(intakeRotationPosition);
        intakeClaw.setPosition(intakeClawPosition);
        intakeCoaxialLeft.setPosition(coaxialPositionGeneral);
        intakeCoaxialRight.setPosition(coaxialPositionGeneral);
        intake4BarLeft.setPosition(fourBarPositionGeneral);
        intake4BarRight.setPosition(fourBarPositionGeneral);
        leftOuttakeArm.setPosition(armPositionGeneral);
        rightOuttakeArm.setPosition(armPositionGeneral);
        outtakeClaw.setPosition(outtakeClawPosition);

        rightHang.setPower(hangPower);
//        leftHang.setPower(hangPower);

        telemetry.addData("Hang Position: ", analogInput.getVoltage() / 3.3 * 360);
        telemetry.update();
    }
}
