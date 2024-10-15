package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class OuttakeArmSubsystem extends SubsystemBase {

    private RobotHardware robot;

    public OuttakeArmSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    public void outtakeArmTransfer() {
        robot.leftOuttakeArm.setPosition(0);
        robot.rightOuttakeArm.setPosition(0);
    }
    public void outtakeArmExit() {
        robot.leftOuttakeArm.setPosition(1);
        robot.rightOuttakeArm.setPosition(1);
    }
    public void outtakeArmHighest() {
        robot.leftOuttakeArm.setPosition(0.75);
        robot.rightOuttakeArm.setPosition(0.75);
    }

}