// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.simulation.SimNotifier;
import frc.robot.simulation.SimRobotState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class FlywheelIOSim extends FlywheelIOHardware {
    public FlywheelIOSim(SimRobotState simRobotState) {
        var gearbox = DCMotor.getKrakenX60Foc(2);
        var plant = LinearSystemId.createFlywheelSystem(gearbox, moi.in(KilogramSquareMeters), gearReduction);
        var motorSim = new FlywheelSim(plant, gearbox);

        var leaderSimState = leaderMotor.getSimState();
        var followerSimState = followerMotor.getSimState();

        SimNotifier.register(deltaTime -> {
            motorSim.setInputVoltage(leaderSimState.getMotorVoltage());
            motorSim.update(deltaTime.in(Seconds));

            var velocity = motorSim.getAngularVelocity();

            leaderSimState.addRotorPosition(velocity.times(gearReduction).times(deltaTime));
            leaderSimState.setRotorVelocity(velocity.times(gearReduction));
            leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            followerSimState.addRotorPosition(velocity.times(gearReduction).times(deltaTime));
            followerSimState.setRotorVelocity(velocity.times(gearReduction));
            followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            simRobotState.setFlywheelVelocity(velocity);
        });
    }
}
