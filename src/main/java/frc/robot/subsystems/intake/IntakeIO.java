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
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputs {
        public Angle rollerPosition = Radians.zero();
        public AngularVelocity rollerVelocity = RadiansPerSecond.zero();
        public Voltage rollerVoltage = Volts.zero();
        public Current rollerStatorCurrent = Amps.zero();
        public Current rollerSupplyCurrent = Amps.zero();
        public double rollerTemperature = 0.0;

        public Angle pivotAngle = Radians.zero();
        public Angle pivotPosition = Radians.zero();
        public AngularVelocity pivotVelocity = RadiansPerSecond.zero();
        public Voltage pivotVoltage = Volts.zero();
        public Current pivotStatorCurrent = Amps.zero();
        public Current pivotSupplyCurrent = Amps.zero();
        public double pivotTemperature = 0.0;
        public Angle pivotClosedLoopReference = Radians.zero();
    }

    public default void updateInputs(IntakeInputs inputs) {}

    public default void setRollerOutput(double value) {}

    public default void setPivotOutput(double value) {}

    public default void setPivotAngle(Angle position) {}

    public default void setPivotNeutralMode(NeutralModeValue mode) {}

    public default void stop() {}
}
