package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class IntakeCoaxialSubsystem extends SubsystemBase {

    private final ServoImplEx coaxialLeft, coaxialRight;
    public Globals.IntakeCoaxialState intakeCoaxialState = Globals.IntakeCoaxialState.REST;

    public IntakeCoaxialSubsystem(ServoImplEx coaxialLeftInput, ServoImplEx coaxialRightInput) {
        coaxialLeft = coaxialLeftInput;
        coaxialRight = coaxialRightInput;
    }

    public void intakeCoaxialCustom(double coaxialInput) {
        coaxialLeft.setPosition(coaxialInput);
        coaxialRight.setPosition(coaxialInput);
        intakeCoaxialState = Globals.IntakeCoaxialState.CUSTOM;
    }

    public void update(Globals.IntakeCoaxialState intakeCoaxialStateInput) {
        intakeCoaxialState = intakeCoaxialStateInput;
        switch (intakeCoaxialStateInput) {
            case REST:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_RESTING);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_RESTING);
                break;
            case BETWEEN:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_BETWEEN);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_BETWEEN);
                break;
            case TRANSFER:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_TRANSFER);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_TRANSFER);
                break;
            case INTAKE:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_INTAKE);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_INTAKE);
                break;
            case CAMERA_READING:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_CAMERA_READING);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_CAMERA_READING);
                break;
        }
    }

}