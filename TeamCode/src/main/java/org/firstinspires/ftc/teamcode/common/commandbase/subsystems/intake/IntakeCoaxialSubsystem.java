package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class IntakeCoaxialSubsystem extends SubsystemBase {

    private final Servo coaxialLeft, coaxialRight;
    public Globals.IntakeCoaxialState intakeCoaxialState = Globals.IntakeCoaxialState.REST;

    public IntakeCoaxialSubsystem(Servo coaxialLeftInput, Servo coaxialRightInput) {
        coaxialLeft = coaxialLeftInput;
        coaxialRight = coaxialRightInput;
    }

    public void update(Globals.IntakeCoaxialState intakeCoaxialStateInput) {
        intakeCoaxialState = intakeCoaxialStateInput;
        switch (intakeCoaxialStateInput) {
            case REST:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_RESTING);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_RESTING);
                break;
            case TRANSFER:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_TRANSFER);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_TRANSFER);
                break;
            case INTAKE:
                coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_INTAKE);
                coaxialRight.setPosition(Globals.INTAKE_COAXIAL_INTAKE);
                break;
        }
    }

}