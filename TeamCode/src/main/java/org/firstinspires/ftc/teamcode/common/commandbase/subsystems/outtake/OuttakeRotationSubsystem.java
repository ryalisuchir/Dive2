package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeRotationSubsystem extends SubsystemBase {

    private final Servo outtakeRotation;
    public Globals.OuttakeRotationState outtakeRotationState = Globals.OuttakeRotationState.TRANSFER;

    public OuttakeRotationSubsystem(Servo outtakeRotationInput) {
        outtakeRotation = outtakeRotationInput;
    }

    public void update (Globals.OuttakeRotationState outtakeRotationStateInput) {
        outtakeRotationState = outtakeRotationStateInput; //Sets global position to the new outtake

        switch (outtakeRotationStateInput) {
            case TRANSFER:
                outtakeRotation.setPosition(Globals.OUTTAKE_ROTATION_TRANSFER);
                break;
            case EXIT:
                outtakeRotation.setPosition(Globals.OUTTAKE_ROTATION_EXIT);
                break;
        }
    }

//    public void outtakeRotationExit() {
//        outtakeRotation.setPosition(0);
//    }
//    public void outtakeRotationTransfer() {
//        outtakeRotation.setPosition(1);
//    }

}