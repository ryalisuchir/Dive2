package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class OuttakeRotationSubsystem extends SubsystemBase {

    private RobotHardware robot;

    public OuttakeRotationSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    public void outtakeRotationExit() {
        robot.outtakeRotation.setPosition(0);
    }
    public void outtakeRotationTransfer() {
        robot.outtakeRotation.setPosition(1);
    }

}