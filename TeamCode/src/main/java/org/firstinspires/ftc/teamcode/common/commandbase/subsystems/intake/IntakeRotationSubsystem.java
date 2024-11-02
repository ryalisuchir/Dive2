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

    public void update(Globals.IntakeRotationState intakeRotationStateInput) {
        intakeRotationState = intakeRotationStateInput;
        switch (intakeRotationStateInput) {
            case REST:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_REST);
                break;
            case TRANSFER:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_TRANSFER);
                break;
            case AUTO_1:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_AUTO_1);
                break;
            case AUTO_2:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_AUTO_2);
                break;
            case AUTO_3:
                intakeRotation.setPosition(Globals.INTAKE_ROTATION_AUTO_3);
                break;
        }
    }

}