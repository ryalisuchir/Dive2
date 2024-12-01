package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.auto.Globals.ExtendoState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.auto.Globals.ExtendoState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.auto.Globals.ExtendoState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    public DcMotorEx extendoMotor;
    private double extendoTargetPosition = 0;
    Globals.ExtendoState extendoState;

    public ExtendoSubsystem(DcMotorEx extendoMotorInput) {
        extendoMotor = extendoMotorInput;
        extendoMotor.setCurrentAlert(Globals.extendoStaticMax, CurrentUnit.AMPS);
    }

    public void currentLoop() {
        if (extendoMotor.isOverCurrent() && extendoMotor.getCurrentPosition() < extendoTargetPosition) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_EXTEND; //Hit another robot, will try again and then park if it doesn't work
            extendoState = REST;
        } else if (extendoMotor.isOverCurrent() && extendoMotor.getCurrentPosition() > extendoTargetPosition) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_RETRACT; //Internal problem
            extendoState = REST;
        }
    }

    public void extendoSlidesLoop(double powerInput) {
        double target = extendoTargetPosition;
        double error = target - extendoMotor.getCurrentPosition();
        extendoMotor.setPower(error * powerInput);

//        if (extendoMotor.getCurrentPosition() < 0)  {
//            extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            extendoTargetPosition = 0;
//        }
    }

    public void extendoManualControlLoop(double joystickInput) {
        if (
                extendoMotor.getCurrentPosition() > Globals.EXTENDO_MAX_EXTENSION - Globals.EXTENDO_MAX_TOLERANCE ||
                        extendoMotor.getCurrentPosition() < Globals.EXTENDO_MAX_RETRACTION + Globals.EXTENDO_MAX_TOLERANCE
        ) {
            extendoMotor.setPower(0);
        } else {
            extendoMotor.setPower(joystickInput);
        }
        if (extendoMotor.getCurrentPosition() < 0) {
            extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoTargetPosition = 0;
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