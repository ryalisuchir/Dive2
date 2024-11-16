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
        double p = 0.025;
        double target = slidesTargetPosition;
        double error = target - leftLift.getCurrentPosition();
        leftLift.setPower(error * p);
        rightLift.setPower(error * p);
//        if (outtakeState == EXTENDING) {
//            if (leftLift.getCurrentPosition() < slidesTargetPosition && Math.abs(slidesTargetPosition - leftLift.getCurrentPosition()) > 5) {
//                leftLift.setPower(1);
//                rightLift.setPower(1);
//            } else if (leftLift.getCurrentPosition() > slidesTargetPosition && Math.abs(slidesTargetPosition - leftLift.getCurrentPosition()) > 5) {
//                leftLift.setPower(-0.3);
//                rightLift.setPower(-0.3);
//            } else if (Math.abs(slidesTargetPosition - leftLift.getCurrentPosition()) < 5) {
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//            }
//        } else if (outtakeState == RETRACTING) {
//            if (leftLift.getCurrentPosition() < slidesTargetPosition && Math.abs(slidesTargetPosition - leftLift.getCurrentPosition()) > 5) {
//                leftLift.setPower(0.3);
//                rightLift.setPower(0.3);
//            } else if (leftLift.getCurrentPosition() > slidesTargetPosition && Math.abs(slidesTargetPosition - leftLift.getCurrentPosition()) > 5) {
//                leftLift.setPower(-1);
//                rightLift.setPower(-1);
//            } else if (Math.abs(slidesTargetPosition - leftLift.getCurrentPosition()) < 5) {
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//            }
//        } else if (outtakeState == REST) {
//            leftLift.setPower(0);
//            rightLift.setPower(0);
//        }
    }

    public void outtakeRetract() {
        slidesTargetPosition = Globals.EXTENDO_MAX_RETRACTION;
        outtakeState = RETRACTING;
    }

    public void outtakeSpecimenExtend() {
        slidesTargetPosition = Globals.LIFT_SPECIMEN_POS;
        outtakeState = EXTENDING;
    }

    public void outtakeMaxExtend() {
        slidesTargetPosition = Globals.EXTENDO_MAX_EXTENSION;
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