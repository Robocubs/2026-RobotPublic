// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IndexerIO {
    @AutoLog
    public class IndexerInputs {
        public Angle position = Radians.zero();
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Voltage voltage = Volts.zero();
        public Current statorCurrent = Amps.zero();
        public Current supplyCurrent = Amps.zero();
        public double temperature = 0.0;
        public AngularVelocity closedLoopReference = RadiansPerSecond.zero();
    }

    public default void updateInputs(IndexerInputs inputs) {}

    public default void setOutput(double percentOutput) {}

    public default void setVelocity(AngularVelocity velocity) {}

    public default void setVoltage(Voltage voltage) {}

    public default void setBrownoutProtection(boolean enabled) {}

    public default void stop() {}
}
