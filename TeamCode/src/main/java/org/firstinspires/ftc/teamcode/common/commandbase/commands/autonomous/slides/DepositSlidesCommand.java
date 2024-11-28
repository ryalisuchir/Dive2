package org.firstinspires.ftc.teamcode.common.commandbase.commands.autonomous.slides;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class DepositSlidesCommand extends CommandBase {
    DepositSubsystem depositSubsystem;
    double depositHeight;

    public DepositSlidesCommand(DepositSubsystem depositSubsystemInput, double heightInput) {
        this.depositSubsystem = depositSubsystemInput;
        this.depositHeight = heightInput;
        addRequirements(depositSubsystem);
    }

    @Override
    public void initialize(){
        depositSubsystem.outtakeSetPosition(depositHeight);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(depositSubsystem.rightLift.getCurrentPosition() - depositHeight) < 20);
    }

    @Override
    public void end(boolean wasInterrupted) {
        if (depositHeight == Globals.LIFT_RETRACT_POS) {
            depositSubsystem.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            depositSubsystem.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
