package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class DepositSubsystem extends SubsystemBase {
    public final DcMotorEx leftLift, rightLift;

    private double slidesTargetPosition = 0;
    Globals.OuttakeState outtakeState;

    public DepositSubsystem(DcMotorEx depoLeftInput, DcMotorEx depoRightInput) {
        leftLift = depoLeftInput;
        rightLift = depoRightInput;
    }

    public void outtakeSlidesLoop(double powerInput) {
        double target = slidesTargetPosition;
        double error = target - rightLift.getCurrentPosition();

        leftLift.setPower(error * powerInput);
        rightLift.setPower(error * powerInput);

        if (rightLift.getCurrentPosition() < 0)  {
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidesTargetPosition = 0;
        }

    }

    public void depositManualControlLoop(double joystickInput) {
        if (
                rightLift.getCurrentPosition() > Globals.LIFT_HIGH_POS - Globals.LIFT_MAX_TOLERANCE ||
                rightLift.getCurrentPosition() < Globals.LIFT_RETRACT_POS + Globals.LIFT_MAX_TOLERANCE
        ) {
            rightLift.setPower(0);
            leftLift.setPower(0);
        } else {
            rightLift.setPower(joystickInput);
            leftLift.setPower(joystickInput);
        }

        if (rightLift.getCurrentPosition() < 0)  {
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesTargetPosition = 0;
        }
    }

    public void outtakeRetract() {
        slidesTargetPosition = Globals.LIFT_RETRACT_POS;
        outtakeState = RETRACTING;
    }

    public void outtakeSpecimenExtend() {
        slidesTargetPosition = Globals.LIFT_SPECIMEN_POS;
        outtakeState = EXTENDING;
    }

    public void outtakeMaxExtend() {
        slidesTargetPosition = Globals.LIFT_HIGH_POS;
        outtakeState = EXTENDING;
    }

    public void outtakeSetPosition(double customSlidesPosition) {
        slidesTargetPosition = customSlidesPosition;
        if (customSlidesPosition > rightLift.getCurrentPosition()) {
            outtakeState = EXTENDING;
        } else if (customSlidesPosition < rightLift.getCurrentPosition()) {
            outtakeState = RETRACTING;
        } else if (customSlidesPosition == rightLift.getCurrentPosition()) {
            outtakeState = REST;
        }
    }

}