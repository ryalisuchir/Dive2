package org.firstinspires.ftc.teamcode.common.commandbase.commands.utility;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class UninterruptableCommand extends CommandBase { //i freaking love saket and arush
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