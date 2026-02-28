// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimNotifier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.indexer.IndexerConstants.*;

public class IndexerIOSim extends IndexerIOHardware {
    public IndexerIOSim() {
        var gearbox = DCMotor.getKrakenX60Foc(1);
        var plant = LinearSystemId.createDCMotorSystem(gearbox, 0.001, gearReduction);
        var motorSim = new DCMotorSim(plant, gearbox);

        var motorSimState = motor.getSimState();

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(motorSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());
            var position = Radians.of(motorSim.getAngularPositionRad());

            motorSimState.setRawRotorPosition(position.times(gearReduction));
            motorSimState.setRotorVelocity(velocity.times(gearReduction));
            motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
