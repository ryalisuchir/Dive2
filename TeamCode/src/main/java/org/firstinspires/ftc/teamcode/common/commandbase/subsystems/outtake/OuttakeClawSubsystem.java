package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeClawState.CLOSED;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeClawState.OPEN;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeClawState.OPEN_TRANSFER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class OuttakeClawSubsystem extends SubsystemBase {

    private RobotHardware robot;

    public OuttakeClawSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    public void outtakeGrab() {
        robot.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_CLOSED);
        Globals.OuttakeClawState outtakeClawState = CLOSED;
    }
    public void outtakeTransfer() {
        robot.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_TRANSFER);
        Globals.OuttakeClawState outtakeClawState = OPEN_TRANSFER;
    }
    public void outtakeRelease() {
        robot.outtakeClaw.setPosition(Globals.OUTTAKE_CLAW_OPEN);
        Globals.OuttakeClawState outtakeClawState = OPEN;
    }

}