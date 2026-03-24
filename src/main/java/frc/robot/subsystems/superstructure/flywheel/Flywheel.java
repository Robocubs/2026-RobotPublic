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
import frc.robot.util.PowerLogger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class Flywheel {
    private final FlywheelIO io;
    private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
    private final PowerLogger leaderPowerLogger = new PowerLogger("FlywheelLeader");
    private final PowerLogger followerPowerLogger = new PowerLogger("FlywheelFollower");

    private Optional<AngularVelocity> lastVelocityControl = Optional.empty();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        leaderPowerLogger.log(inputs.leaderStatorCurrent, inputs.leaderVoltage);
        followerPowerLogger.log(inputs.followerStatorCurrent, inputs.followerVoltage);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
        lastVelocityControl = Optional.of(velocity);
        Logger.recordOutput("Flywheel/TargetVelocity", velocity);
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
        lastVelocityControl = Optional.empty();
        Logger.recordOutput("Flywheel/TargetVelocity", RadiansPerSecond.zero());
    }

    public void stop() {
        io.stop();
        lastVelocityControl = Optional.empty();
        Logger.recordOutput("Flywheel/TargetVelocity", RadiansPerSecond.zero());
    }

    @AutoLogOutput
    public boolean isAtGoal() {
        return lastVelocityControl.isPresent() && isAtVelocity(lastVelocityControl.get());
    }

    public boolean isAtVelocity(AngularVelocity target) {
        return target.isNear(inputs.leaderVelocity, velocityTolerance.get());
    }
}
