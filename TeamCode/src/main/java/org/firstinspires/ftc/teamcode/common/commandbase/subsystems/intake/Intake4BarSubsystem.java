package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class Intake4BarSubsystem extends SubsystemBase {

    private final Servo intake4BarLeft, intake4BarRight;
    public Globals.FourBarState fourBarState = Globals.FourBarState.RESTING;

    public Intake4BarSubsystem(Servo intake4BarLeftInput, Servo intake4BarRightInput) {
        intake4BarLeft = intake4BarLeftInput;
        intake4BarRight = intake4BarRightInput;
    }

    public void intake4BarIntake() {
        intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
        intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
    }

    public void intake4BarScanning() {
        intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
        intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
    }

    public void intake4BarLow() {
        intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_LOW);
        intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_LOW);
    }

    public void intake4BarTransfer() {
        intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
        intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
    }

    public void intake4BarResting() {
        intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_RESTING);
        intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_RESTING);
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
        }
    }


}