package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class HangSubsystem extends SubsystemBase {

    private final CRServo leftHang;
    private final CRServo rightHang;
    private final AnalogInput analogInput;

    public HangSubsystem(CRServo leftHangInput, CRServo rightHangInput, AnalogInput hangInput) {
        this.leftHang = leftHangInput;
        this.rightHang = rightHangInput;
        this.analogInput = hangInput;
    }

    public void setServoPower(double power) {
        leftHang.setPower(power);
        rightHang.setPower(power);
    }

    public void stopServos() {
        leftHang.setPower(0);
        rightHang.setPower(0);
    }

    public double getHangPosition() {
        return analogInput.getVoltage() / 3.3 * 360;
    }
}