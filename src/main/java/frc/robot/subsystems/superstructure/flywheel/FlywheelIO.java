// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelInputs {
        public Angle leaderPosition = Radians.zero();
        public AngularVelocity leaderVelocity = RadiansPerSecond.zero();
        public Voltage leaderVoltage = Volts.zero();
        public Current leaderStatorCurrent = Amps.zero();
        public Current leaderSupplyCurrent = Amps.zero();
        public double leaderTemperature = 0.0;

        public Angle followerPosition = Radians.zero();
        public AngularVelocity followerVelocity = RadiansPerSecond.zero();
        public Voltage followerVoltage = Volts.zero();
        public Current followerStatorCurrent = Amps.zero();
        public Current followerSupplyCurrent = Amps.zero();
        public double followerTemperature = 0.0;

        public AngularVelocity closedLoopReference = RadiansPerSecond.zero();
    }

    public default void updateInputs(FlywheelInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}
}
