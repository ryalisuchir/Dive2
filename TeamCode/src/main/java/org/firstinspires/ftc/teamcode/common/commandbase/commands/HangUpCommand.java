package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.HangSubsystem;

public class HangUpCommand extends CommandBase {

    private final HangSubsystem hangUpSubsystem;
    private final double power;
    private final long duration;
    private long startTime;

    public HangUpCommand(HangSubsystem hangUpSubsystem, double power, long duration) {
        this.hangUpSubsystem = hangUpSubsystem;
        this.power = power;
        this.duration = duration;
        addRequirements(hangUpSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        hangUpSubsystem.setServoPower(power);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        hangUpSubsystem.stopServos();
    }
}
