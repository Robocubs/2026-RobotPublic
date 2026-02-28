// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

public class CommandLogger {
    private static CommandLogger mInstance = null;

    private final Map<String, Integer> commandTotalCounts = new HashMap<>(100);
    private final Map<String, Integer> commandCounts = new HashMap<>(100);
    private final List<String> commandEvents = new ArrayList<>(100);

    public static CommandLogger getInstance() {
        if (mInstance == null) {
            synchronized (CommandLogger.class) {
                if (mInstance == null) {
                    mInstance = new CommandLogger();
                }
            }
        }

        return mInstance;
    }

    private CommandLogger() {
        CommandScheduler.getInstance().onCommandInitialize(this::logInitialized);
        CommandScheduler.getInstance().onCommandFinish(this::logFinished);
        CommandScheduler.getInstance().onCommandInterrupt(this::logInterrupted);
    }

    public void logInitialized(Command command) {
        logCommand(command, true, "Initialized");
    }

    public void logFinished(Command command) {
        logCommand(command, false, "Finished");
    }

    public void logInterrupted(Command command) {
        logCommand(command, false, "Interrupted");
    }

    public void periodic() {
        Logger.recordOutput("Commands/Events", commandEvents.toArray(String[]::new));
        commandEvents.clear();
    }

    private void logCommand(Command command, boolean active, String event) {
        var name = command.getName();
        var count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
        var totalCount = commandTotalCounts.getOrDefault(name, 0) + (active ? 1 : 0);
        commandCounts.put(name, count);
        commandTotalCounts.put(name, totalCount);

        var hash = Integer.toHexString(command.hashCode());
        Logger.recordOutput("Commands/" + name + "/Active/" + hash, active);
        Logger.recordOutput("Commands/" + name + "/ActiveCount", count);
        Logger.recordOutput("Commands/" + name + "/TotalCount", totalCount);

        commandEvents.add(name + "_" + hash + "_" + event);
    }
}
