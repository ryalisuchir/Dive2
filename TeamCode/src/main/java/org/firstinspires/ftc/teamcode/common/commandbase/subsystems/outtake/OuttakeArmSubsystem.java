package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class OuttakeArmSubsystem extends SubsystemBase {
    public final ServoImplEx leftOuttakeArm, rightOuttakeArm;
    public Globals.OuttakeArmState armState = Globals.OuttakeArmState.TRANSFER;

    public OuttakeArmSubsystem(ServoImplEx leftOuttakeArmInput, ServoImplEx rightOuttakeArmInput) {
        leftOuttakeArm = leftOuttakeArmInput;
        rightOuttakeArm = rightOuttakeArmInput;
    }

    public void outtakeArmCustom(double outtakeInput) {
        leftOuttakeArm.setPosition(outtakeInput);
        rightOuttakeArm.setPosition(outtakeInput);
        armState = Globals.OuttakeArmState.CUSTOM;
    }

    public void update(Globals.OuttakeArmState outtakeArmStateInput) {
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
            case SPECIMEN_INTAKE:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_INTAKE);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_INTAKE);
                break;
            case SPECIMEN_OUTTAKE:
                leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_LOWER);
                rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_LOWER);
                break;
        }
    }

}