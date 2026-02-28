// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.hood.HoodConstants.*;

public class Hood {
    public static final LoggedTunableNumber kG =
            new LoggedTunableNumber("Hood/kG", Constants.robot == RobotType.SIM_BOT ? 0.002 : 0.085);
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 4.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0.05);

    private final HoodIO io;
    private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    private final PIDController pidController;

    private @AutoLogOutput @Getter Angle measuredAngle = Radians.zero();
    private @AutoLogOutput @Getter AngularVelocity measuredVelocity = RadiansPerSecond.zero();
    private Optional<Angle> lastPositionControl = Optional.empty();

    public Hood(HoodIO io) {
        this.io = io;

        pidController = new PIDController(kP.get(), 0.0, kD.get());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        measuredAngle = Radians.of(
                MathUtil.angleModulus(inputs.position.in(Radians) / encoderReduction + angleAtEncoderZero.in(Radians)));
        measuredVelocity = RadiansPerSecond.of(inputs.velocity.in(RadiansPerSecond) / encoderReduction);

        LoggedTunableValue.ifChanged(
                0,
                () -> {
                    pidController.setP(kP.get());
                    pidController.setD(kD.get());
                },
                kP,
                kD);
    }

    public void setAngle(Angle angle) {
        var clampedAngleRadians = MathUtil.clamp(angle.in(Radians), minAngle.in(Radians), maxAngle.in(Radians));

        var output = MathUtil.isNear(
                        measuredAngle.in(Radians),
                        clampedAngleRadians,
                        angleTolerance.get().in(Radians) * 0.5)
                ? 0
                : pidController.calculate(measuredAngle.in(Radians), clampedAngleRadians);
        var kg = kG.get() * Math.cos(clampedAngleRadians);

        output = MathUtil.clamp(output + kg, -1.0, 1.0);
        io.setServoOutput(output);

        var targetAngle = Radians.of(clampedAngleRadians);
        lastPositionControl = Optional.of(targetAngle);
        Logger.recordOutput("Hood/TargetAngle", targetAngle);
        Logger.recordOutput("Hood/Output", output);
    }

    public void setServoOutput(double output) {
        output = MathUtil.clamp(output, -1.0, 1.0);
        io.setServoOutput(output);
        lastPositionControl = Optional.empty();
        Logger.recordOutput("Hood/TargetAngle", Radians.zero());
        Logger.recordOutput("Hood/Output", output);
    }

    public void stop() {
        io.stop();
        lastPositionControl = Optional.empty();
        Logger.recordOutput("Hood/TargetAngle", Radians.zero());
        Logger.recordOutput("Hood/Output", 0.0);
    }

    public Angle getAngle() {
        return measuredAngle;
    }

    @AutoLogOutput
    public boolean isAtGoal() {
        return lastPositionControl.isPresent() && isAtAngle(lastPositionControl.get());
    }

    public boolean isAtAngle(Angle target) {
        return measuredAngle.isNear(target, angleTolerance.get());
    }
}
