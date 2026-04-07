// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;

public class CompositePowerLogger {
    private final String powerKey;
    private final String powerConsumptionKey;
    private final PowerLogger[] loggers;

    public CompositePowerLogger(String key, PowerLogger... loggers) {
        this.powerKey = "PowerLogger/Power/" + key;
        this.powerConsumptionKey = "PowerLogger/PowerConsumption/" + key;
        this.loggers = loggers;
    }

    public void log() {
        var power = 0.0;
        var totalPowerConsumption = 0.0;
        for (var logger : loggers) {
            power += logger.getPower();
            totalPowerConsumption += logger.getTotalPowerConsumption();
        }
        Logger.recordOutput(powerKey, power);
        Logger.recordOutput(powerConsumptionKey, totalPowerConsumption);
    }
}
