package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class IntakeRotationSubsystem extends SubsystemBase {

    private final Servo intakeRotation;
    public Globals.IntakeRotationState intakeRotationState = Globals.IntakeRotationState.TRANSFER;

    public IntakeRotationSubsystem(Servo intakeRotationInput) {
        intakeRotation = intakeRotationInput;
    }

    public void intakeRotationRest() {
        intakeRotation.setPosition(Globals.INTAKE_ROTATION_REST);
        intakeRotationState = Globals.IntakeRotationState.REST;
    }

    public void intakeRotationTransfer() {
        intakeRotation.setPosition(Globals.INTAKE_ROTATION_TRANSFER);
        intakeRotationState = Globals.IntakeRotationState.TRANSFER;
    }

    public void intakeRotationAutoCustom(double customRotationInput) {
        intakeRotation.setPosition(customRotationInput);
        intakeRotationState = Globals.IntakeRotationState.CUSTOM_AUTO;
    }

    public void update(Globals.IntakeRotationState intakeRotationStateInput, double rotationPosition) {
        intakeRotationState = intakeRotationStateInput;
        switch (intakeRotationStateInput) {
            case REST:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_REST);
                break;
            case TRANSFER:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_TRANSFER);
                break;
            case CUSTOM_AUTO:
                intakeRotation.setPosition(rotationPosition);
                break;
        }
    }

}