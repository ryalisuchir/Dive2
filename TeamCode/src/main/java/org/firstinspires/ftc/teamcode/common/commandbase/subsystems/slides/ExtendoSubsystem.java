package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    private DcMotorEx extendoMotor;
    private double extendoTargetPosition = 0;
    Globals.ExtendoState extendoState;

    public ExtendoSubsystem(DcMotorEx extendoMotorInput) {
        extendoMotor = extendoMotorInput;
        extendoMotor.setCurrentAlert(Globals.extendoStaticMax, CurrentUnit.AMPS);
    }

    public void currentLoop() {
        if(extendoMotor.isOverCurrent() && extendoMotor.getCurrentPosition() < extendoTargetPosition) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_EXTEND; //Hit another robot, will try again and then park if it doesn't work
            extendoState = REST;
        } else if(extendoMotor.isOverCurrent() && extendoMotor.getCurrentPosition() > extendoTargetPosition) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_RETRACT; //Internal problem
            extendoState = REST;
        }
    }

    public void extendoSlidesLoop() {
        if (extendoState == EXTENDING) {
            if ((extendoMotor.getCurrentPosition() < extendoTargetPosition) && (Math.abs(extendoTargetPosition - extendoMotor.getCurrentPosition()) > 5)) {
                extendoMotor.setPower(1);
            } else if (extendoMotor.getCurrentPosition() > extendoTargetPosition && (Math.abs(extendoTargetPosition - extendoMotor.getCurrentPosition()) > 5)) {
                extendoMotor.setPower(-0.3);
            } else if (Math.abs(extendoTargetPosition - extendoMotor.getCurrentPosition()) < 5) {
                extendoMotor.setPower(0);
            } else if (extendoState == RETRACTING) {
                if (extendoMotor.getCurrentPosition() < extendoTargetPosition && (Math.abs(extendoTargetPosition - extendoMotor.getCurrentPosition()) > 5)) {
                    extendoMotor.setPower(0.3);
                } else if (extendoMotor.getCurrentPosition() > extendoTargetPosition && (Math.abs(extendoTargetPosition - extendoMotor.getCurrentPosition()) > 5)) {
                    extendoMotor.setPower(-1);
                } else if (Math.abs(extendoTargetPosition - extendoMotor.getCurrentPosition()) < 5) {
                    extendoMotor.setPower(0);
                }
            } else if (extendoState == REST) {
                    extendoMotor.setPower(0);
                }
        }
    }

    public void extendoRetract() {
        extendoTargetPosition = Globals.EXTENDO_MAX_RETRACTION;
        extendoState = RETRACTING;
    }

    public void extendoMaxExtend() {
        extendoTargetPosition = Globals.EXTENDO_MAX_EXTENSION;
        extendoState = EXTENDING;
    }

    public void extendoSetPosition(double customSlidesPosition) {
        extendoTargetPosition = customSlidesPosition;
        if (customSlidesPosition > extendoMotor.getCurrentPosition()) {
            extendoState = EXTENDING;
        } else if (customSlidesPosition < extendoMotor.getCurrentPosition()) {
            extendoState = RETRACTING;
        } else if (customSlidesPosition == extendoMotor.getCurrentPosition()) {
            extendoState = REST;
        }
    }

}