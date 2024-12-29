package org.firstinspires.ftc.teamcode.common.commandbase.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public class DeferredCommand implements Command {
    private final Supplier<Command> commandSupplier;
    private final Set<Subsystem> requirements;
    private Command currentCommand;

    public DeferredCommand(Supplier<Command> commandSupplier, Subsystem... requirements) {
        this.commandSupplier = commandSupplier;
        this.requirements = new HashSet<>();
        for (Subsystem subsystem : requirements) {
            this.requirements.add(subsystem);
        }
    }

    @Override
    public void initialize() {
        currentCommand = commandSupplier.get();
        if (currentCommand != null) {
            currentCommand.initialize();
        } else {
            throw new IllegalStateException("DeferredCommand: Supplied command was null!");
        }
    }

    @Override
    public void execute() {
        if (currentCommand != null) {
            currentCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
            currentCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand != null && currentCommand.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }
}
