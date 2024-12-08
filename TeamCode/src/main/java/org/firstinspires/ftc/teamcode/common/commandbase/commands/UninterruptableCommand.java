package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class UninterruptableCommand extends CommandBase {
    private final Command command;

    public UninterruptableCommand(Command command) {
        this.command = command;
    }

    @Override
    public void initialize() {
        command.schedule(false);
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(command);
    }
}