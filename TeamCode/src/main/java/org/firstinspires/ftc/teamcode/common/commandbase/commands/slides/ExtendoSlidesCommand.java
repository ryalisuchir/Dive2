package org.firstinspires.ftc.teamcode.common.commandbase.commands.slides;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;

public class ExtendoSlidesCommand extends CommandBase {
    ExtendoSubsystem extendoSubsystem;
    double extendoLength;

    public ExtendoSlidesCommand(ExtendoSubsystem extendoSubsystemInput, double extendoInput) {
        this.extendoSubsystem = extendoSubsystemInput;
        this.extendoLength = extendoInput;
        addRequirements(extendoSubsystem);
    }

    @Override
    public void initialize() {
        extendoSubsystem.extendoSetPosition(extendoLength);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(extendoSubsystem.extendoMotor.getCurrentPosition() - extendoLength) < Globals.EXTENDO_MAX_TOLERANCE);
    }
}
