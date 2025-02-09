package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.IntakeClawState.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class IntakeClawSubsystem extends SubsystemBase {

    public final ServoImplEx intakeClaw;
    public Globals.IntakeClawState clawState = OPEN;

    public IntakeClawSubsystem(ServoImplEx intakeClawInput) {
        intakeClaw = intakeClawInput;
    }

    public void intakeClawOpen() {
        intakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
        clawState = Globals.IntakeClawState.OPEN;
    }

    public void intakeClawClosed() {
        intakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
        clawState = Globals.IntakeClawState.CLOSED;
    }

    public void update(Globals.IntakeClawState intakeClawState) {
        clawState = intakeClawState;
        switch (intakeClawState) {
            case OPEN:
                intakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                intakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
                break;
            case OPEN_TRANSFER:
                intakeClaw.setPosition(Globals.INTAKE_CLAW_TRANSFER);
                break;
        }
    }
}