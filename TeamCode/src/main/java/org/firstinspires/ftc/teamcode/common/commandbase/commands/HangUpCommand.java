package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.HangSubsystem;

public class HangUpCommand extends CommandBase {

    private final HangSubsystem hangUpSubsystem;
    private final double position;

    public HangUpCommand(HangSubsystem hangUpSubsystem, double position) {
        this.hangUpSubsystem = hangUpSubsystem;
        this.position = position;
        addRequirements(hangUpSubsystem);
    }

    @Override
    public void initialize() {
        hangUpSubsystem.stopServos(); // Ensure servos are stopped initially
    }

    @Override
    public void execute() {
        double currentPosition = hangUpSubsystem.getHangPosition();

        if (currentPosition < position) {
            hangUpSubsystem.setServoPower(1); // Move up
        } else if (currentPosition > position) {
            hangUpSubsystem.setServoPower(-1); // Move down
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hangUpSubsystem.getHangPosition() - position) < 5;
    }

    @Override
    public void end(boolean interrupted) {
        hangUpSubsystem.stopServos();
    }
}
