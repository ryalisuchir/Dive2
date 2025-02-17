package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    public static double p = 0.017;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    private static final PIDFController extendoPIDF = new PIDFController(p, i, d, f);
    public static double setPoint = 0;
    public static double maxPowerConstant = 1;
    public DcMotorEx extendoMotor;
    public ElapsedTime timer = new ElapsedTime();
    Globals.ExtendoState extendoState;
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

    public void extendoSlidesLoop(double currentP) {
        timer.reset();

        motorPos = extendoMotor.getCurrentPosition();

        extendoPIDF.setP(currentP * p);
        extendoPIDF.setI(i);
        extendoPIDF.setD(d);
        extendoPIDF.setF(f);

        extendoPIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPos) + maxPowerConstant;
        double power = Range.clip(extendoPIDF.calculate(motorPos, setPoint), -maxPower, maxPower);

        extendoMotor.setPower(power);
    }

    public void extendoSlidesLoop() {
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