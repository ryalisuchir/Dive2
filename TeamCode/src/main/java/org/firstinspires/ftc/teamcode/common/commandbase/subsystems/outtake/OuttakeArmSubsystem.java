package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeClawState.CLOSED;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeArmSubsystem extends SubsystemBase {
    private final ServoImplEx leftOuttakeArm, rightOuttakeArm;
    public Globals.OuttakeArmState armState = Globals.OuttakeArmState.TRANSFER;

    public OuttakeArmSubsystem(ServoImplEx leftOuttakeArmInput, ServoImplEx rightOuttakeArmInput) {
        leftOuttakeArm = leftOuttakeArmInput;
        rightOuttakeArm = rightOuttakeArmInput;
    }

    public void outtakeArmTransfer() {
        leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
        rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
        armState = Globals.OuttakeArmState.TRANSFER;
    }
    public void outtakeArmBucket() {
        leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
        rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
        armState = Globals.OuttakeArmState.BUCKET;
    }
    public void outtakeArmDunk() {
        leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_DUNK);
        rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_DUNK);
        armState = Globals.OuttakeArmState.DUNK;
    }
    public void outtakeArmRaising() {
        leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_RAISING);
        rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_RAISING);
        armState = Globals.OuttakeArmState.RAISING;
    }
    public void outtakeArmSpecimen() {
        leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN);
        rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN);
        armState = Globals.OuttakeArmState.SPECIMEN;
    }

    public void update (Globals.OuttakeArmState outtakeArmStateInput) {
        armState = outtakeArmStateInput; //Sets global position to the new outtake
        switch (outtakeArmStateInput) {
            case TRANSFER:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                break;
            case BUCKET:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
                break;
            case DUNK:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_DUNK);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_DUNK);
                break;
            case RAISING:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_RAISING);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_RAISING);
                break;
            case SPECIMEN:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN);
                break;
        }
    }

}