package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeRotationSubsystem extends SubsystemBase {

    private final ServoImplEx outtakeRotation;
    public Globals.OuttakeRotationState outtakeRotationState = Globals.OuttakeRotationState.TRANSFER;

    public OuttakeRotationSubsystem(ServoImplEx outtakeRotationInput) {
        outtakeRotation = outtakeRotationInput;
    }

    public void outtakeRotationExit() {
        outtakeRotation.setPosition(Globals.OUTTAKE_ROTATION_EXIT);
        outtakeRotationState = Globals.OuttakeRotationState.EXIT;
    }
    public void outtakeRotationSpecimen() {
        outtakeRotation.setPosition(Globals.OUTTAKE_ROTATION_SPECIMEN);
        outtakeRotationState = Globals.OuttakeRotationState.SPECIMEN;
    }
    public void outtakeRotationTransfer() {
        outtakeRotation.setPosition(Globals.OUTTAKE_ROTATION_TRANSFER);
        outtakeRotationState = Globals.OuttakeRotationState.TRANSFER;
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

}