package org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OuttakeArmSubsystem extends SubsystemBase {
    private final Servo leftOuttakeArm, rightOuttakeArm;

    public OuttakeArmSubsystem(Servo leftOuttakeArmInput, Servo rightOuttakeArmInput) {
        leftOuttakeArm = leftOuttakeArmInput;
        rightOuttakeArm = rightOuttakeArmInput;
    }
    public void outtakeArmTransfer() {
        leftOuttakeArm.setPosition(0);
        rightOuttakeArm.setPosition(0);
    }
    public void outtakeArmExit() {
        leftOuttakeArm.setPosition(1);
        rightOuttakeArm.setPosition(1);
    }
    public void outtakeArmHighest() {
        leftOuttakeArm.setPosition(0.75);
        rightOuttakeArm.setPosition(0.75);
    }

}