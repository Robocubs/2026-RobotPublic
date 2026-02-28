// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface TurretIO {
    @AutoLog
    public class TurretInputs {
        public double timestamp = 0.0;

        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage voltage = Volts.zero();
        public Current statorCurrent = Amps.zero();
        public Current supplyCurrent = Amps.zero();
        public double temperature = 0.0;
        public Angle closedLoopReference = Radians.zero();

        public boolean encoder1Connected;
        public boolean encoder2Connected;
        public Angle encoder1Position = Radians.zero();
        public Angle encoder2Position = Radians.zero();
    }

    public default void updateInputs(TurretInputs inputs) {}

    public default void setPosition(Angle angle) {}

    public default void setPosition(Angle angle, AngularVelocity velocity) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}

    public default void resetPosition(Angle angle) {}
}
