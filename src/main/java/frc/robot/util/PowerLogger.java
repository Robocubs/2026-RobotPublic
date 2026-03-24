// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class PowerLogger {
    private final String powerKey;
    private final String powerConsumptionKey;
    private double totalPowerConsumption = 0.0;
    private double lastTimestamp = Timer.getTimestamp();

    public PowerLogger(String key) {
        this.powerKey = "PowerLogger/Power/" + key;
        this.powerConsumptionKey = "PowerLogger/PowerConsumption/" + key;
    }

    public void log(Current current, Voltage voltage) {
        var timestamp = Timer.getTimestamp();
        var power = Math.abs(current.in(Amps) * voltage.in(Volts));
        totalPowerConsumption += power * (timestamp - lastTimestamp);
        lastTimestamp = timestamp;
        Logger.recordOutput(powerKey, power);
        Logger.recordOutput(powerConsumptionKey, totalPowerConsumption);
    }
}
