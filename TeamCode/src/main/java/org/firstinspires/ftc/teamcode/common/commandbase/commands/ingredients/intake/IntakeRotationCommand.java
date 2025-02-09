package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeRotationSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class IntakeRotationCommand extends CommandBase {
    IntakeRotationSubsystem intakeRotationSubsystem;
    Globals.IntakeRotationState intakeRotationState;
    private double setPosition;

    public IntakeRotationCommand(IntakeRotationSubsystem intakeRotationSubsystemInput, Globals.IntakeRotationState intakeRotationStateInput) {
        this.intakeRotationSubsystem = intakeRotationSubsystemInput;
        this.intakeRotationState = intakeRotationStateInput;

        addRequirements(intakeRotationSubsystem);
    }

    public IntakeRotationCommand(IntakeRotationSubsystem intakeRotationSubsystemInput, double rotationInput) {
        this.intakeRotationSubsystem = intakeRotationSubsystemInput;
        this.intakeRotationState = Globals.IntakeRotationState.CUSTOM;
        setPosition = rotationInput;

        addRequirements(intakeRotationSubsystem);
    }

    @Override
    public void initialize() {
        Log.i("IntakeRotation Command: ", intakeRotationState.toString());
        switch (intakeRotationState) {
            case REST:
                setPosition = Globals.INTAKE_ROTATION_REST;
                intakeRotationSubsystem.intakeRotation.setPosition(Globals.INTAKE_ROTATION_REST);
                break;
            case TRANSFER:
                setPosition = Globals.INTAKE_ROTATION_TRANSFER;
                intakeRotationSubsystem.intakeRotation.setPosition(Globals.INTAKE_ROTATION_TRANSFER);
                break;
            case CUSTOM:
                intakeRotationSubsystem.intakeRotation.setPosition(setPosition);
                break;
        }
        Log.i("IntakeRotation Value: ", String.valueOf(intakeRotationSubsystem.intakeRotation.getPosition()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
