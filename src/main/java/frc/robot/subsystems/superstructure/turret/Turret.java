// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.turret;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.PhysicsUtil;
import frc.robot.util.PowerLogger;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class Turret {
    private final TurretIO io;
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
    private final PowerLogger powerLogger = new PowerLogger("Turret");
    private final EasyCRT easyCRT =
            new EasyCRT(new EasyCRTConfig(() -> inputs.encoder1Position, () -> inputs.encoder2Position)
                    .withCommonDriveGear(commonGearReduction, commonGearTeeth, cancoder1Teeth, cancoder2Teeth)
                    .withAbsoluteEncoderOffsets(cancoder1Offset, cancoder2Offset)
                    .withMechanismRange(crtMinAngle, crtMaxAngle)
                    .withMatchTolerance(crtMatchTolerance)
                    .withAbsoluteEncoderInversions(encoder1Inverted, encoder2Inverted));

    private @AutoLogOutput boolean hasBeenZeroed = false;
    private double lastZeroedMeasuredError = Double.POSITIVE_INFINITY;
    private Optional<Angle> lastPositionControl = Optional.empty();
    private @AutoLogOutput @Getter boolean isWrapping = false;

    public static record TurretSetpoint(Angle position, AngularVelocity velocity) {}

    public Turret(TurretIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        powerLogger.log(inputs.statorCurrent, inputs.voltage);

        var crtStart = Timer.getFPGATimestamp();
        var crtResult = easyCRT.getAngleOptional();
        var crtDuration = Timer.getFPGATimestamp() - crtStart;

        // Reset position if encoders are connected, CRT has a valid result, and
        // A. we have not previously reset position, or
        // B. robot is disabled and the CRT result is lower error than previous results
        if (inputs.encoder1Connected
                && inputs.encoder2Connected
                && crtResult.isPresent()
                && (!hasBeenZeroed
                        || DriverStation.isDisabled() && easyCRT.getLastErrorRotations() < lastZeroedMeasuredError)) {
            io.resetPosition(crtResult.get());
            inputs.position = crtResult.get();
            hasBeenZeroed = true;
            lastZeroedMeasuredError = easyCRT.getLastErrorRotations();
        }

        Logger.recordOutput("Turret/CrtStatus", easyCRT.getLastStatus());
        Logger.recordOutput("Turret/CrtCalculationMS", crtDuration * 1000);
        if (crtResult.isPresent()) {
            Logger.recordOutput("Turret/CrtResult", crtResult.get().in(Radians));
        }
        if (!Double.isNaN(easyCRT.getLastErrorRotations())) {
            Logger.recordOutput("Turret/CrtError", Units.rotationsToRadians(easyCRT.getLastErrorRotations()));
        }
    }

    public void setAngleUnoptimized(Angle angle) {
        io.setPosition(angle);
        lastPositionControl = Optional.of(angle);
        isWrapping = false;
    }

    public void setAngle(Angle angle) {
        setAngle(angle, RotationsPerSecond.zero());
    }

    public void setAngle(Rotation2d rotation, AngularVelocity velocity) {
        setAngle(Radians.of(rotation.getRadians()), velocity);
    }

    public void setAngle(TurretSetpoint setpoint) {
        setAngle(setpoint.position, setpoint.velocity);
    }

    public void setAngle(Angle angleRadians, AngularVelocity velocity) {
        var targetRadians = optimizeAngle(angleRadians.in(Radians));
        var target = Radians.of(targetRadians);
        if (!MathUtil.isNear(targetRadians, inputs.position.in(Radians), Math.PI / 4.0)) {
            isWrapping = true;
        } else if (isAtAngle(lastPositionControl.orElse(target))) {
            isWrapping = false;
        }

        io.setPosition(target, velocity);
        lastPositionControl = Optional.of(target);
    }

    private double optimizeAngle(double angleRadians) {
        if (maxRangeLessThanOneRotation) {
            return MathUtil.inputModulus(angleRadians, midAngle.in(Radians) - Math.PI, midAngle.in(Radians) + Math.PI);
        }

        var lastPositionControlRadians =
                lastPositionControl.orElseGet(() -> inputs.position).in(Radians);
        var targetRadians =
                lastPositionControlRadians + MathUtil.angleModulus(angleRadians - lastPositionControlRadians);

        while (true) {
            if (targetRadians >= maxAngle.in(Radians)) {
                targetRadians -= 2 * Math.PI;
            } else if (targetRadians <= minAngle.in(Radians)) {
                targetRadians += 2 * Math.PI;
            } else {
                break;
            }
        }

        Logger.recordOutput("Turret/UnoptimizedAngle", angleRadians);
        Logger.recordOutput("Turret/OptimizedAngle", targetRadians);

        return targetRadians;
    }

    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage);
        lastPositionControl = Optional.empty();
    }

    public void stop() {
        io.stop();
        lastPositionControl = Optional.empty();
        isWrapping = false;
    }

    public void resetPosition() {
        io.resetPosition(Radians.zero());
        lastPositionControl = Optional.empty();
    }

    public Angle getPosition() {
        return PhysicsUtil.predictCurrentValue(inputs.position, inputs.velocity, inputs.timestamp);
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    @AutoLogOutput
    public boolean isAtGoal() {
        return lastPositionControl.isPresent() && isAtAngle(lastPositionControl.get());
    }

    public boolean isAtAngle(Angle target) {
        return inputs.position.isNear(target, angleTolerance.get());
    }
}
