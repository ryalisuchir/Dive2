package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class IntakeRotationSubsystem extends SubsystemBase {

    private RobotHardware robot;

    public IntakeRotationSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    public void intakeRotationReset() {
        robot.intakeRotation.setPosition(0.5);
    }
    public void intakeRotation(Double rotationAngle) {
        robot.intakeRotation.setPosition(rotationAngle);
    }

}