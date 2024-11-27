package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class IntakeRotationSubsystem extends SubsystemBase {

    private final ServoImplEx intakeRotation;
    public Globals.IntakeRotationState intakeRotationState = Globals.IntakeRotationState.TRANSFER;

    public IntakeRotationSubsystem(ServoImplEx intakeRotationInput) {
        intakeRotation = intakeRotationInput;
    }

    public void intakeRotationCustom(double customRotationInput) {
        intakeRotation.setPosition(customRotationInput);
        intakeRotationState = Globals.IntakeRotationState.CUSTOM;
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
            case CUSTOM:
                intakeRotation.setPosition(rotationPosition);
                break;
        }
    }

}