package org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.regular;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
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
    public void initialize(){
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
            case SPECIMEN:
                setPosition = Globals.OUTTAKE_ARM_SPECIMEN;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN);
                break;
            case SPECIMEN_INTAKE:
                setPosition = Globals.OUTTAKE_ARM_INTAKE;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_INTAKE);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_INTAKE);
                break;
            case SPECIMEN_OUTTAKE:
                setPosition = Globals.OUTTAKE_ARM_SPECIMEN_LOWER;
                outtakeArmSubsystem.leftOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_LOWER);
                outtakeArmSubsystem.rightOuttakeArm.setPosition(Globals.OUTTAKE_ARM_SPECIMEN_LOWER);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(setPosition - outtakeArmSubsystem.leftOuttakeArm.getPosition()) < 0.0001);
    }

}
