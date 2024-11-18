package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeClawSubsystem extends SubsystemBase {

    private final ServoImplEx outtakeClaw;
    public Globals.OuttakeClawState clawState = Globals.OuttakeClawState.OPEN;

    public OuttakeClawSubsystem(ServoImplEx outtakeClawInput) {
        outtakeClaw = outtakeClawInput;
    }

    public void outtakeClawOpen() {
        outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
        clawState = Globals.OuttakeClawState.OPEN;
    }

    public void outtakeClawClosed() {
        outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_CLOSED);
        clawState = Globals.OuttakeClawState.CLOSED;
    }

    public void outtakeClawTransfer() {
        outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_TRANSFER);
        clawState = Globals.OuttakeClawState.OPEN_TRANSFER;
    }

    public void update(Globals.OuttakeClawState outtakeClawState) {
        clawState = outtakeClawState;
        switch (outtakeClawState) {
            case OPEN:
                outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_CLOSED);
                break;
            case OPEN_TRANSFER:
                outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_TRANSFER);
                break;
        }
    }
}