package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class OuttakeArmCommand extends CommandBase {
    OuttakeArmSubsystem outtakeArmSubsystem;
    Globals.OuttakeArmState outtakeArmState;
    double setPosition;

    public OuttakeArmCommand(OuttakeArmSubsystem outtakeArmSubsystemInput, Globals.OuttakeArmState outtakeArmStateInput) {
        this.outtakeArmSubsystem = outtakeArmSubsystemInput;
        this.outtakeArmState = outtakeArmStateInput;

        addRequirements(outtakeArmSubsystem);
    }

    @Override
    public void initialize() {
        switch (outtakeArmState) {
            case TRANSFER:
                setPosition = Globals.OUTTAKE_ARM_TRANSFER;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_TRANSFER);
                break;
            case BUCKET:
                setPosition = Globals.OUTTAKE_ARM_BUCKET;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_BUCKET);
                break;
            case DUNK:
                setPosition = Globals.OUTTAKE_ARM_DUNK;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_DUNK);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_DUNK);
                break;
            case RAISING:
                setPosition = Globals.OUTTAKE_ARM_RAISING;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_RAISING);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_RAISING);
                break;
            case SPECIMEN_INTAKE:
                setPosition = Globals.OUTTAKE_ARM_SPECIMEN_INTAKE;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_INTAKE);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_INTAKE);
                break;
            case SPECIMEN_OUTTAKE:
                setPosition = Globals.OUTTAKE_ARM_SPECIMEN_DROPOFF;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_DROPOFF);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_DROPOFF);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(setPosition - outtakeArmSubsystem.leftOuttakeArm.getPosition()) < 0.0001);
    }

}
