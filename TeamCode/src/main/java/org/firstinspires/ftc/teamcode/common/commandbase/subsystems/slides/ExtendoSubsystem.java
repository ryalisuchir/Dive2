package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.EXTENDING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoState.RETRACTING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    private RobotHardware robot;
    private double extendoTargetPosition = 0;
    Globals.ExtendoState extendoState;

    public ExtendoSubsystem(RobotHardware robot) {
        this.robot = robot;
        robot.extendoMotor.setCurrentAlert(Globals.extendoStaticMax, CurrentUnit.AMPS);
    }

    public void currentLoop() {
        if(robot.extendoMotor.isOverCurrent() && robot.extendoMotor.getCurrentPosition() < extendoTargetPosition) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_EXTEND; //Hit another robot, will try again and then park if it doesn't work
            extendoState = REST;
        } else if(robot.extendoMotor.isOverCurrent() && robot.extendoMotor.getCurrentPosition() > extendoTargetPosition) {
            Globals.ExtendoFailState extendoFailState = Globals.ExtendoFailState.FAILED_RETRACT; //Internal problem
            extendoState = REST;
        }
    }

    public void extendoUpdate() {
        if (extendoState == EXTENDING) {
            if (robot.extendoMotor.getCurrentPosition() < extendoTargetPosition) {
                robot.extendoMotor.setPower(1);
            } else if (robot.extendoMotor.getCurrentPosition() > extendoTargetPosition) {
                robot.extendoMotor.setPower(-0.3);
            }
        } else if (extendoState == RETRACTING) {
            if (robot.extendoMotor.getCurrentPosition() < extendoTargetPosition) {
                robot.extendoMotor.setPower(0.3);
            } else if (robot.extendoMotor.getCurrentPosition() > extendoTargetPosition) {
                robot.extendoMotor.setPower(-1);
            }
        } else if (extendoState == REST) {
            robot.extendoMotor.setPower(0);
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
        if (customSlidesPosition > robot.extendoMotor.getCurrentPosition()) {
            extendoState = EXTENDING;
        } else if (customSlidesPosition < robot.extendoMotor.getCurrentPosition()) {
            extendoState = RETRACTING;
        } else if (customSlidesPosition == robot.extendoMotor.getCurrentPosition()) {
            extendoState = REST;
        }
    }

}