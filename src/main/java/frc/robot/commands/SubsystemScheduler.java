// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SubsystemScheduler<Subsystem extends SubsystemBase> {
    private final Subsystem subsystem;
    private Command defaultCommand;
    private Command command;
    private Command nextCommand = null;

    public SubsystemScheduler(Subsystem subsystem, Command defaultCommand) {
        this.subsystem = subsystem;

        CommandScheduler.getInstance().registerComposedCommands(defaultCommand);
        this.defaultCommand = defaultCommand;
        command = this.defaultCommand;
    }

    public final void initialize() {
        command = this.defaultCommand;
        command.initialize();
    }

    public final void execute() {
        if (nextCommand != null) {
            command.end(true);
            command = nextCommand;
            command.initialize();
            nextCommand = null;
        }

        command.execute();

        if (command.isFinished()) {
            command.end(false);
            command = defaultCommand;
            command.initialize();
        }

        Logger.recordOutput("SubsystemScheduler/" + subsystem.getName() + "/Command", command.getName());
    }

    public final void end(boolean interrupted) {
        command.end(interrupted);
    }

    public Command cmd() {
        return new FunctionalCommand(this::initialize, this::execute, this::end, () -> false, subsystem);
    }

    public final Command schedule(Function<Subsystem, Command> command) {
        var commandValue = command.apply(subsystem);
        CommandScheduler.getInstance().registerComposedCommands(commandValue);
        return Commands.runOnce(() -> nextCommand = commandValue);
    }

    public Subsystem getSubsystem() {
        return subsystem;
    }
}
