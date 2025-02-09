package org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.transfer.ground.utility;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.ExtendoSubsystem;

public class IntakeSliderResetterCommand extends CommandBase {

    private final ExtendoSubsystem extendoSubsystem;
    private long startTime;

    public IntakeSliderResetterCommand(ExtendoSubsystem extendoSubsystem) {
        this.extendoSubsystem = extendoSubsystem;
        addRequirements(extendoSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        extendoSubsystem.extendoMotor.setPower(-1);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= 3000;
    }

    @Override
    public void end(boolean interrupted) {
        extendoSubsystem.extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoSubsystem.extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoSubsystem.extendoMotor.setPower(0);
    }
}
