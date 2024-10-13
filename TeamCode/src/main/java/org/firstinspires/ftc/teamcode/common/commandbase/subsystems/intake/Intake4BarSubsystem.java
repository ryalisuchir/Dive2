package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class Intake4BarSubsystem extends SubsystemBase {

    private RobotHardware robot;
    public Globals.FourBarState fourBarState = Globals.FourBarState.RESTING;

    public Intake4BarSubsystem(RobotHardware robot) {
        this.robot = robot;
        update(Globals.FourBarState.TRANSFER);
    }

    public void update(Globals.FourBarState fourState) {
        fourBarState = fourState;
        switch (fourState) {
            case INTAKE:
                robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
                break;
            case SCANNING:
                robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_SCANNING);
                break;
            case LOW:
                robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_LOW);
            case TRANSFER:
                robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
            case RESTING:
                robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_RESTING);
        }
    }

//    public void fourBarRest() {
//        robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_RESTING);
//    }
//    public void fourBarIntake() {
//        robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_INTAKE);
//    }
//    public void fourBarLowIntake() {
//        robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_LOW);
//    }
//    public void fourBarTransfer() {
//        robot.intake4Bar.setPosition(Globals.INTAKE_FOURBAR_TRANSFER);
//    }

}