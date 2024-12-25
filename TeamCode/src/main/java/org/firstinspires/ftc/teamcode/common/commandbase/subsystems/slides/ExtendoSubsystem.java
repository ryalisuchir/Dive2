package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    public DcMotorEx extendoMotor;
    Globals.ExtendoState extendoState;

    public static double p = 0.0245;
    public static double i = 0;
    public static double d = 0.0003;
    public static double f = 0;

    public static double setPoint = 0;
    public static double maxPowerConstant = 1;

    private static final PIDFController extendoPIDF = new PIDFController(p,i,d, f);

    public ElapsedTime timer = new ElapsedTime();

    int motorPos = 0;

    public ExtendoSubsystem(DcMotorEx extendoMotorInput) {
        extendoMotor = extendoMotorInput;
        extendoMotor.setCurrentAlert(Globals.extendoStaticMax, CurrentUnit.AMPS);
    }

    public void currentLoop() {
        if (extendoMotor.isOverCurrent() && extendoMotor.getCurrentPosition() < setPoint) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_EXTEND; //Hit another robot, will try again and then park if it doesn't work
            extendoState = REST;
        } else if (extendoMotor.isOverCurrent() && extendoMotor.getCurrentPosition() > setPoint) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_RETRACT; //Internal problem
            extendoState = REST;
        }
    }

    public void extendoSlidesLoop(double powerInput) {
        timer.reset();

        motorPos = extendoMotor.getCurrentPosition();

        extendoPIDF.setP(p);
        extendoPIDF.setI(i);
        extendoPIDF.setD(d);
        extendoPIDF.setF(f);

        extendoPIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPos) + maxPowerConstant;
        double power = Range.clip(extendoPIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

        extendoMotor.setPower(power);

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
            setPoint = 0;
        }
    }

    public void extendoRetract() {
        setPoint = Globals.EXTENDO_MAX_RETRACTION;
        extendoState = RETRACTING;
    }

    public void extendoMaxExtend() {
        setPoint = Globals.EXTENDO_MAX_EXTENSION;
        extendoState = EXTENDING;
    }

    public void extendoSetPosition(double customSlidesPosition) {
        setPoint = customSlidesPosition;
        if (customSlidesPosition > extendoMotor.getCurrentPosition()) {
            extendoState = EXTENDING;
        } else if (customSlidesPosition < extendoMotor.getCurrentPosition()) {
            extendoState = RETRACTING;
        } else if (customSlidesPosition == extendoMotor.getCurrentPosition()) {
            extendoState = REST;
        }
    }

}