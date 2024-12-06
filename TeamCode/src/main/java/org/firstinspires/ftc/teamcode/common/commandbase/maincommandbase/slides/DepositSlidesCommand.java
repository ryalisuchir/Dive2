package org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;

public class DepositSlidesCommand extends CommandBase {
    DepositSubsystem depositSubsystem;
    public double depositHeight;

    public DepositSlidesCommand(DepositSubsystem depositSubsystemInput, double heightInput) {
        this.depositSubsystem = depositSubsystemInput;
        this.depositHeight = heightInput;
        addRequirements(depositSubsystem);
    }

    @Override
    public void initialize() {
        depositSubsystem.outtakeSetPosition(depositHeight);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(depositSubsystem.rightLift.getCurrentPosition() - depositHeight) < 45);
    }

}
