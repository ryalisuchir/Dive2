package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeCoaxialSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class IntakeCoaxialCommand extends CommandBase {
    IntakeCoaxialSubsystem intakeCoaxialSubsystem;
    Globals.IntakeCoaxialState coaxialState;
    private double setPosition;

    public IntakeCoaxialCommand(IntakeCoaxialSubsystem intakeCoaxialSubsystemInput, Globals.IntakeCoaxialState coaxialStateInput) {
        this.intakeCoaxialSubsystem = intakeCoaxialSubsystemInput;
        this.coaxialState = coaxialStateInput;

        addRequirements(intakeCoaxialSubsystem);
    }

    @Override
    public void initialize() {
        Log.i("IntakeCoaxial Command: ", coaxialState.toString());
        switch (coaxialState) {
            case INTAKE:
                setPosition = Globals.INTAKE_COAXIAL_INTAKE;
                intakeCoaxialSubsystem.coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_INTAKE);
                intakeCoaxialSubsystem.coaxialRight.setPosition(Globals.INTAKE_COAXIAL_INTAKE);
                break;
            case TRANSFER:
                setPosition = Globals.INTAKE_COAXIAL_TRANSFER;
                intakeCoaxialSubsystem.coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_TRANSFER);
                intakeCoaxialSubsystem.coaxialRight.setPosition(Globals.INTAKE_COAXIAL_TRANSFER);
                break;
            case CAMERA_READING:
                setPosition = Globals.INTAKE_COAXIAL_CAMERA_READING;
                intakeCoaxialSubsystem.coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_CAMERA_READING);
                intakeCoaxialSubsystem.coaxialRight.setPosition(Globals.INTAKE_COAXIAL_CAMERA_READING);
                break;
            case REST:
                setPosition = Globals.INTAKE_COAXIAL_RESTING;
                intakeCoaxialSubsystem.coaxialLeft.setPosition(Globals.INTAKE_COAXIAL_RESTING);
                intakeCoaxialSubsystem.coaxialRight.setPosition(Globals.INTAKE_COAXIAL_RESTING);
                break;
        }
        Log.i("IntakeCoaxial Value: ", String.valueOf(intakeCoaxialSubsystem.coaxialLeft.getPosition()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
