package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class HangSubsystem extends SubsystemBase {

    private final CRServo leftHang;
    private final CRServo rightHang;

    public HangSubsystem(CRServo leftHangInput, CRServo rightHangInput) {
        this.leftHang = leftHangInput;
        this.rightHang = rightHangInput;
    }

    public void setServoPower(double power) {
        leftHang.setPower(power);
        rightHang.setPower(power);
    }

    public void stopServos() {
        leftHang.setPower(0);
        rightHang.setPower(0);
    }
}