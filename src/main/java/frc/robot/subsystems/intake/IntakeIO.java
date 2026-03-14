// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputs {
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

        public Distance extensionPosition = Meters.zero();
        public LinearVelocity extensionVelocity = MetersPerSecond.zero();
        public Voltage extensionVoltage = Volts.zero();
        public Current extensionStatorCurrent = Amps.zero();
        public Current extensionSupplyCurrent = Amps.zero();
        public double extensionTemperature = 0.0;
        public Distance extensionClosedLoopReference = Meters.zero();
    }

    public default void updateInputs(IntakeInputs inputs) {}

    public default void setRollerOutput(double value) {}

    public default void setExtensionOutput(double value) {}

    public default void setExtensionOutput(double value, boolean ignoreLimits) {}

    public default void setExtensionPosition(Distance position) {}

    public default void setExtensionNeutralMode(NeutralModeValue mode) {}

    public default void setExtensionVoltage(Voltage voltage) {}

    public default void setExtensionEncoderPosition(Distance position) {}

    public default void stop() {}
}
