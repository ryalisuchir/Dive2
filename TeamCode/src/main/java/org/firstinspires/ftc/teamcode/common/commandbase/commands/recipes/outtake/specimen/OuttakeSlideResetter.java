package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.outtake.specimen;

import android.webkit.WebStorage;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.ExtendoSubsystem;

public class OuttakeSlideResetter extends CommandBase {

    private final DepositSubsystem depositSubsystem;
    private long startTime;

    public OuttakeSlideResetter(DepositSubsystem depositSubsystem) {
        this.depositSubsystem = depositSubsystem;
        addRequirements(depositSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        depositSubsystem.rightLift.setPower(-1);
        depositSubsystem.leftLift.setPower(-1);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= 1000;
    }

    @Override
    public void end(boolean interrupted) {
        depositSubsystem.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositSubsystem.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depositSubsystem.rightLift.setPower(0);
        depositSubsystem.leftLift.setPower(0);
    }
}
