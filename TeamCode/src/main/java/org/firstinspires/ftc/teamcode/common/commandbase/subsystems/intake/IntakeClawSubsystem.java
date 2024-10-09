package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.IntakeClawState.OPEN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class IntakeClawSubsystem extends SubsystemBase {

    private RobotHardware robot;
    public Globals.IntakeClawState clawState = OPEN;

    public IntakeClawSubsystem(RobotHardware robot) {
       this.robot = robot;
       update(Globals.IntakeClawState.OPEN);
    }

    public void update (Globals.IntakeClawState intakeClawState) {
        clawState = intakeClawState;
        switch (intakeClawState) {
            case OPEN:
                robot.intakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                robot.intakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
                break;
            case OPEN_TRANSFER:
                robot.intakeClaw.setPosition(Globals.INTAKE_CLAW_TRANSFER);
        }
    }

//    public void intakeGrab() {
//       robot.intakeClaw.setPosition(Globals.INTAKE_CLAW_CLOSED);
//    }
//    public void intakeRelease() {
//        robot.intakeClaw.setPosition(Globals.INTAKE_CLAW_OPEN);
//    }
//    public void intakeTransferRelease() {
//        robot.intakeClaw.setPosition(Globals.INTAKE_CLAW_TRANSFER);
//    }

}