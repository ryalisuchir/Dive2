package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeClawSubsystem extends SubsystemBase {

    private final Servo outtakeClaw;
    public Globals.OuttakeClawState clawState = Globals.OuttakeClawState.CLOSED;

    public OuttakeClawSubsystem(Servo outtakeClawInput) {
        outtakeClaw = outtakeClawInput;
    }

    public void update (Globals.OuttakeClawState outtakeClawState) {
        clawState = outtakeClawState; //Sets global position to the new outtake

        switch (outtakeClawState) {
            case OPEN:
                outtakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                outtakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
                break;
            case OPEN_TRANSFER:
                outtakeClaw.setPosition(Globals.INTAKE_CLAW_TRANSFER);
                break;
        }
    }

    //In case I was just being dumb and need to go back to this (in which case I will be very upset smh):
//    public void outtakeGrab() {
//        outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_CLOSED);
//        Globals.OuttakeClawState outtakeClawState = CLOSED;
//    }
//    public void outtakeTransfer() {
//        outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_TRANSFER);
//        Globals.OuttakeClawState outtakeClawState = OPEN_TRANSFER;
//    }
//    public void outtakeRelease() {
//        outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
//        Globals.OuttakeClawState outtakeClawState = OPEN;
//    }

}