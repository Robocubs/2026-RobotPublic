// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class LoggedCommand extends WrapperCommand {
    private final Command mCommand;

    LoggedCommand(Command command) {
        super(command);
        mCommand = command;
    }

    @Override
    public void initialize() {
        super.initialize();
        CommandLogger.getInstance().logInitialized(mCommand);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            CommandLogger.getInstance().logInterrupted(mCommand);
        } else {
            CommandLogger.getInstance().logFinished(mCommand);
        }
    }
}
