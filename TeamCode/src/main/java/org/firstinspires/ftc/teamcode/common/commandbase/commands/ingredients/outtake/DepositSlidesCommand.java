package org.firstinspires.ftc.teamcode.common.commandbase.commands.ingredients.outtake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class DepositSlidesCommand extends CommandBase {
    public double depositHeight;
    DepositSubsystem depositSubsystem;

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
        return (Math.abs(depositSubsystem.rightLift.getCurrentPosition() - depositHeight) < Globals.LIFT_MAX_TOLERANCE);
    }

}
