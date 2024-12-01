package org.firstinspires.ftc.teamcode.common.commandbase.maincommandbase.slides;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.ExtendoSubsystem;

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
        return (Math.abs(extendoSubsystem.extendoMotor.getCurrentPosition() - extendoLength) < 35);
    }
}
