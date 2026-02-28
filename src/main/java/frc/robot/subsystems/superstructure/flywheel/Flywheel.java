// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import java.util.Optional;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class Flywheel {
    private final FlywheelIO io;
    private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
    private Optional<AngularVelocity> lastVelocityControl = Optional.empty();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
        lastVelocityControl = Optional.of(velocity);
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
        lastVelocityControl = Optional.empty();
    }

    public void stop() {
        io.stop();
        lastVelocityControl = Optional.empty();
    }

    @AutoLogOutput
    public boolean isAtGoal() {
        return lastVelocityControl.isPresent() && isAtVelocity(lastVelocityControl.get());
    }

    public boolean isAtVelocity(AngularVelocity target) {
        return target.isNear(inputs.leaderVelocity, velocityTolerance.get());
    }
}
