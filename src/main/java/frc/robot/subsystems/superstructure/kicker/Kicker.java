// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.kicker;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class Kicker {
    private final KickerIO io;
    private final KickerInputsAutoLogged inputs = new KickerInputsAutoLogged();

    public Kicker(KickerIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
    }

    public void setOutput(double output) {
        io.setOutput(output);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.stop();
    }

    public boolean isAtVelocity(AngularVelocity target) {
        return inputs.velocity.isNear(target, KickerConstants.velocityTolerance.get());
    }
}
