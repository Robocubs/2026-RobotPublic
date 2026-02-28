// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command intake() {
        return this.run(() -> {
                    io.setPivotAngle(IntakeConstants.angleDeployed);
                    io.setRollerOutput(1);
                })
                .withName("Intake/Intake");
    }

    public Command reverse() {
        return this.run(() -> {
                    io.setPivotAngle(IntakeConstants.angleDeployed);
                    io.setRollerOutput(-1);
                })
                .withName("Intake/Reverse");
    }

    public Command stow() {
        return this.run(() -> {
                    io.setPivotAngle(IntakeConstants.angleRetracted);
                    io.setRollerOutput(0);
                })
                .withName("Intake/Stow");
    }

    public Command agitate() {
        return this.run(() -> {
                    io.setPivotAngle(IntakeConstants.angleRetracted);
                    io.setRollerOutput(1);
                })
                .withName("Intake/Agitate");
    }

    public Command stop() {
        return this.run(() -> {
                    io.stop();
                })
                .withName("Intake/Stop");
    }
}
