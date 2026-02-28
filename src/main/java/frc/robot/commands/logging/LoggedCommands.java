// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.logging;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LoggedCommands {
    public static Command logged(Command command) {
        return (command instanceof LoggedCommand) ? command : new LoggedCommand(command);
    }

    public static Command loggedSequence(Command... commands) {
        return Commands.sequence(Stream.of(commands).map(LoggedCommands::logged).toArray(Command[]::new));
    }
}
