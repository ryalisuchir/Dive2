package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.IntakeClawState.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class IntakeClawSubsystem extends SubsystemBase {

    private final Servo intakeClaw;
    public Globals.IntakeClawState clawState = OPEN;

    public IntakeClawSubsystem(Servo intakeClawInput) {
        intakeClaw = intakeClawInput;
    }

    public void intakeClawOpen() {
        intakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
    }

    public void intakeClawClosed() {
        intakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
    }

    public void intakeClawTransfer() {
        intakeClaw.setPosition(Globals.INTAKE_CLAW_TRANSFER);
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