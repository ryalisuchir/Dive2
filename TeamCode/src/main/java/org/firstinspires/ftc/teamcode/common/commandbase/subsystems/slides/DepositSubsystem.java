package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class DepositSubsystem extends SubsystemBase {
    public static double p = 0.01;
    public static double i = 0.001;
    public static double d = 0;
    public static double f = 0;
    private static final PIDFController slidePIDF = new PIDFController(p, i, d, f);
    public static double setPoint = 0;
    public static double maxPowerConstant = 1.0;
    public final DcMotorEx leftLift, rightLift;
    public ElapsedTime timer = new ElapsedTime();
    int motorPosition;
    Globals.OuttakeState outtakeState;

    public DepositSubsystem(DcMotorEx depoLeftInput, DcMotorEx depoRightInput) {
        leftLift = depoLeftInput;
        rightLift = depoRightInput;
    }

    public void outtakeSlidesLoop() {
        timer.reset();

        motorPosition = rightLift.getCurrentPosition();

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPosition) + maxPowerConstant;
        double power = Range.clip(slidePIDF.calculate(motorPosition, setPoint), -0.6, maxPower);

        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void outtakeSlidesLoop(double dterm) {
        timer.reset();

        motorPosition = rightLift.getCurrentPosition();

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(dterm);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(setPoint);

        double maxPower = (f * motorPosition) + maxPowerConstant;
        double power = Range.clip(slidePIDF.calculate(motorPosition, setPoint), -0.6, maxPower);

        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void depositManualControlLoop(double joystickInput) {
        if (
                rightLift.getCurrentPosition() > Globals.LIFT_HIGH_POS - Globals.LIFT_MAX_TOLERANCE
        ) {
            rightLift.setPower(0);
            leftLift.setPower(0);
        } else {
            rightLift.setPower(joystickInput);
            leftLift.setPower(joystickInput);
        }
        rightLift.setPower(joystickInput);
        leftLift.setPower(joystickInput);
    }

    public void outtakeSetPosition(double customSlidesPosition) {
        setPoint = customSlidesPosition;
        if (customSlidesPosition > rightLift.getCurrentPosition()) {
            outtakeState = EXTENDING;
        } else if (customSlidesPosition < rightLift.getCurrentPosition()) {
            outtakeState = RETRACTING;
        } else if (customSlidesPosition == rightLift.getCurrentPosition()) {
            outtakeState = REST;
        }
    }
}