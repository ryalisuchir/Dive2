package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class IntakeCoaxialSubsystem extends SubsystemBase { //This will be kept constant at all times.

    private RobotHardware robot;
    public Globals.CoaxialState coaxialState = Globals.CoaxialState.REST;

    public IntakeCoaxialSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    public void update(Globals.CoaxialState coaxialStateA) {
        coaxialState = coaxialStateA;
        robot.intakeCoaxial.setPosition(0);
    }
//    public void coaxialRest() {
//        robot.intakeCoaxial.setPosition(0);
//    }

}