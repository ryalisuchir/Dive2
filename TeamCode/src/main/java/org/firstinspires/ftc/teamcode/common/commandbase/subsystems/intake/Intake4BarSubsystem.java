package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;

@Config
public class Intake4BarSubsystem extends SubsystemBase {

    public final ServoImplEx intake4BarLeft, intake4BarRight;
    public Globals.FourBarState fourBarState = Globals.FourBarState.RESTING;

    public Intake4BarSubsystem(ServoImplEx intake4BarLeftInput, ServoImplEx intake4BarRightInput) {
        intake4BarLeft = intake4BarLeftInput;
        intake4BarRight = intake4BarRightInput;
    }

    public void intake4BarCustom(double fourBarInput) {
        intake4BarLeft.setPosition(fourBarInput);
        intake4BarRight.setPosition(fourBarInput);
        fourBarState = Globals.FourBarState.CUSTOM;
    }

    public void update(Globals.FourBarState fourState) {
        fourBarState = fourState;
        switch (fourState) {
            case INTAKE:
                intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
                intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
                break;
            case SCANNING:
                intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
                intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
                break;
            case LOW:
                intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_LOW);
                intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_LOW);
                break;
            case TRANSFER:
                intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
                intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
                break;
            case RESTING:
                intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_RESTING);
                intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_RESTING);
                break;
            case BETWEEN:
                intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_BETWEEN);
                intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_BETWEEN);
                break;
        }
    }


}