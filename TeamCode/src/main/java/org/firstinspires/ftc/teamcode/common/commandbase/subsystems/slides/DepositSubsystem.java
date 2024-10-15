package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class DepositSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private double slidesTargetPosition = 0;
    Globals.OuttakeState outtakeState;

    public DepositSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void outtakeSlidesUpdate() {
        if (outtakeState == EXTENDING) {
            if (robot.leftLift.getCurrentPosition() < slidesTargetPosition) {
                robot.leftLift.setPower(1);
                robot.rightLift.setPower(1);
            } else if (robot.extendoMotor.getCurrentPosition() > slidesTargetPosition) {
                robot.leftLift.setPower(-0.3);
                robot.rightLift.setPower(-0.3);
            }
        } else if (outtakeState == RETRACTING) {
            if (robot.extendoMotor.getCurrentPosition() < slidesTargetPosition) {
                robot.leftLift.setPower(0.3);
                robot.rightLift.setPower(0.3);
            } else if (robot.extendoMotor.getCurrentPosition() > slidesTargetPosition) {
                robot.leftLift.setPower(-1);
                robot.rightLift.setPower(-1);
            }
        } else if (outtakeState == REST) {
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);
        }
    }

    public void outtakeRetract() {
        slidesTargetPosition = Globals.EXTENDO_MAX_RETRACTION;
        outtakeState = RETRACTING;
    }

    public void outtakeMaxExtend() {
        slidesTargetPosition = Globals.EXTENDO_MAX_EXTENSION;
        outtakeState = EXTENDING;
    }

    public void outtakeSetPosition(double customSlidesPosition) {
        slidesTargetPosition = customSlidesPosition;
        if (customSlidesPosition > robot.extendoMotor.getCurrentPosition()) {
            outtakeState = EXTENDING;
        } else if (customSlidesPosition < robot.extendoMotor.getCurrentPosition()) {
            outtakeState = RETRACTING;
        } else if (customSlidesPosition == robot.extendoMotor.getCurrentPosition()) {
            outtakeState = REST;
        }
    }

}