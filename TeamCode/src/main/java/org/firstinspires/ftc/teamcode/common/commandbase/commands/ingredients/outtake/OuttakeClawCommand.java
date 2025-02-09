package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class OuttakeClawCommand extends CommandBase {
    OuttakeClawSubsystem outtakeClawSubsystem;
    Globals.OuttakeClawState outtakeClawState;
    private double setPosition;

    public OuttakeClawCommand(OuttakeClawSubsystem outtakeClawSubsystemInput, Globals.OuttakeClawState outtakeClawInput) {
        this.outtakeClawSubsystem = outtakeClawSubsystemInput;
        this.outtakeClawState = outtakeClawInput;

        addRequirements(outtakeClawSubsystem);
    }

    @Override
    public void initialize() {
        Log.i("OuttakeClaw Command: ", outtakeClawState.toString());
        switch (outtakeClawState) {
            case OPEN:
                setPosition = Globals.OUTTAKE_CLAW_OPEN;
                outtakeClawSubsystem.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                setPosition = Globals.OUTTAKE_CLAW_CLOSED;
                outtakeClawSubsystem.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_CLOSED);
                break;
        }
        Log.i("OuttakeClaw Value: ", String.valueOf(outtakeClawSubsystem.outtakeClaw.getPosition()));
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(setPosition - outtakeClawSubsystem.outtakeClaw.getPosition()) < 0.0001);
    }
}