package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeClawSubsystem extends SubsystemBase {

    public final ServoImplEx outtakeClaw;
    public Globals.OuttakeClawState clawState = Globals.OuttakeClawState.OPEN;

    public OuttakeClawSubsystem(ServoImplEx outtakeClawInput) {
        outtakeClaw = outtakeClawInput;
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
        }
    }
}