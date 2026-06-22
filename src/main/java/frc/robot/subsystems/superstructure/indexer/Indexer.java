// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PowerLogger;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.indexer.IndexerConstants.velocityTolerance;

public class Indexer {
    private final IndexerIO io;
    private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();
    private final PowerLogger powerLogger = new PowerLogger("Indexer");

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        powerLogger.log(inputs.statorCurrent, inputs.voltage);
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

    public void setBrownoutProtection(boolean enabled) {
        io.setBrownoutProtection(enabled);
    }

    public boolean isAtVelocity(AngularVelocity target) {
        return inputs.velocity.isNear(target, velocityTolerance.get());
    }
}
