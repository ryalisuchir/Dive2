package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;

@Config
public class DepositSubsystem extends SubsystemBase {
    private final DcMotorEx leftLift, rightLift;

    private double slidesTargetPosition = 0;
    Globals.OuttakeState outtakeState;

    public DepositSubsystem(DcMotorEx depoLeftInput, DcMotorEx depoRightInput) {
        leftLift = depoLeftInput;
        rightLift = depoRightInput;
    }

    public void outtakeSlidesLoop() {
        double p = 0.015;
        double target = slidesTargetPosition;
        double error = target - leftLift.getCurrentPosition();
        leftLift.setPower(error * p);
        rightLift.setPower(error * p);
    }

    public void depositManualControlLoop(double joystickInput) {
        if (
            leftLift.getCurrentPosition() > Globals.LIFT_HIGH_POS - Globals.LIFT_MAX_TOLERANCE ||
            leftLift.getCurrentPosition() < Globals.LIFT_RETRACT_POS + Globals.LIFT_MAX_TOLERANCE
        ) {
            rightLift.setPower(0);
            leftLift.setPower(0);
        } else {
            rightLift.setPower(joystickInput);
            leftLift.setPower(joystickInput);
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
        if (customSlidesPosition > leftLift.getCurrentPosition()) {
            outtakeState = EXTENDING;
        } else if (customSlidesPosition < leftLift.getCurrentPosition()) {
            outtakeState = RETRACTING;
        } else if (customSlidesPosition == leftLift.getCurrentPosition()) {
            outtakeState = REST;
        }
    }

}