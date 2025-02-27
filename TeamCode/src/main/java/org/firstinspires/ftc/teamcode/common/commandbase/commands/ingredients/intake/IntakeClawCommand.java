package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.intake;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class IntakeClawCommand extends CommandBase {
    IntakeClawSubsystem intakeClawSubsystem;
    Globals.IntakeClawState intakeClawState;
    private double setPosition;

    public IntakeClawCommand(IntakeClawSubsystem intakeClawSubsystemInput, Globals.IntakeClawState intakeClawInput) {
        this.intakeClawSubsystem = intakeClawSubsystemInput;
        this.intakeClawState = intakeClawInput;

        addRequirements(intakeClawSubsystem);
    }

    @Override
    public void initialize() {
        Log.i("IntakeClaw Command: ", intakeClawState.toString());
        switch (intakeClawState) {
            case OPEN:
                setPosition = Globals.INTAKE_CLAW_OPEN;
                intakeClawSubsystem.intakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                setPosition = Globals.INTAKE_CLAW_CLOSED;
                intakeClawSubsystem.intakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
                break;
        }
        Log.i("IntakeClaw Value: ", String.valueOf(intakeClawSubsystem.intakeClaw.getPosition()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
