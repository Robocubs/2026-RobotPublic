// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ContinuousSwitchCommand extends Command {
    private final List<Option> options = new ArrayList<>();

    private int currentIndex = -1;
    private int lastIndex = -1;
    private boolean ended = false;

    private record Option(BooleanSupplier shouldRun, Command command) {}

    public ContinuousSwitchCommand(Command defaultCommand) {
        CommandScheduler.getInstance().registerComposedCommands(defaultCommand);
        options.add(new Option(() -> true, defaultCommand));
        addRequirements(defaultCommand.getRequirements());
    }

    public ContinuousSwitchCommand withOption(BooleanSupplier shouldRun, Command command) {
        if (currentIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(command);

        options.add(options.size() - 1, new Option(shouldRun, command));
        return this;
    }

    private int determineActiveCommand() {
        for (var i = 0; i < options.size(); i++) {
            if (options.get(i).shouldRun.getAsBoolean()) {
                return i;
            }
        }

        return options.size() - 1;
    }

    @Override
    public final void initialize() {
        ended = false;
        currentIndex = determineActiveCommand();
        lastIndex = currentIndex;
        options.get(currentIndex).command.initialize();
    }

    @Override
    public final void execute() {
        currentIndex = determineActiveCommand();
        var currentCommand = options.get(currentIndex).command;

        if (currentIndex != lastIndex) {
            var lastCommand = options.get(lastIndex).command;
            lastCommand.execute();
            lastCommand.end(true);
            lastIndex = currentIndex;
            ended = false;
            currentCommand.initialize();
            return;
        }

        if (ended) {
            ended = false;
            currentCommand.initialize();
        }

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            ended = true;
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted && currentIndex > -1 && currentIndex < options.size()) {
            options.get(currentIndex).command.end(interrupted);
        }
        currentIndex = -1;
        ended = true;
    }

    @Override
    public final boolean isFinished() {
        return false;
    }
}
