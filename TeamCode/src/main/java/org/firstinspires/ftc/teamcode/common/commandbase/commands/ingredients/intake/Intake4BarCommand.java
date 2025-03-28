package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.Intake4BarSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class Intake4BarCommand extends CommandBase {
    Intake4BarSubsystem intake4BarSubsystem;
    Globals.FourBarState fourBarState;
    double setPosition;

    public Intake4BarCommand(Intake4BarSubsystem intake4BarSubsystemInput, Globals.FourBarState fourBarStateInput) {
        this.intake4BarSubsystem = intake4BarSubsystemInput;
        this.fourBarState = fourBarStateInput;

        addRequirements(intake4BarSubsystem);
    }

    @Override
    public void initialize() {
        Log.i("Intake4Bar Command: ", fourBarState.toString());
        switch (fourBarState) {
            case INTAKE:
                setPosition = Globals.INTAKE_FOURBAR_INTAKE;
                intake4BarSubsystem.intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
                intake4BarSubsystem.intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
                break;
            case SCANNING:
                setPosition = Globals.INTAKE_FOURBAR_SCANNING;
                intake4BarSubsystem.intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
                intake4BarSubsystem.intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
                break;
            case TRANSFER:
                setPosition = Globals.INTAKE_FOURBAR_TRANSFER;
                intake4BarSubsystem.intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
                intake4BarSubsystem.intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
                break;
            case RESTING:
                setPosition = Globals.INTAKE_FOURBAR_RESTING;
                intake4BarSubsystem.intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_RESTING);
                intake4BarSubsystem.intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_RESTING);
                break;
            case CAMERA_READING:
                setPosition = Globals.INTAKE_FOURBAR_CAMERA_READING;
                intake4BarSubsystem.intake4BarLeft.setPosition(Globals.INTAKE_FOURBAR_CAMERA_READING);
                intake4BarSubsystem.intake4BarRight.setPosition(Globals.INTAKE_FOURBAR_CAMERA_READING);
                break;
        }
        Log.i("Intake4Bar Value: ", String.valueOf(intake4BarSubsystem.intake4BarRight.getPosition()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
