// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.characterization.SysIdCommandSet;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Intake extends SubsystemBase {
    private static final LoggedTunableNumber zeroRoutineOutput =
            new LoggedTunableNumber("Intake/ZeroRoutineOutput", 0.1);
    private static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> zeroVelocityTolerance =
            new LoggedTunableMeasure<>("Intake/ZeroVelocityTolerance", MetersPerSecond.of(0.01));

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private @AutoLogOutput Distance extensionGoal = IntakeConstants.minimumExtension;

    public Intake(IntakeIO io) {
        this.io = io;

        RobotState.getInstance().setHopperExtensionSupplier(() -> inputs.extensionPosition);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    private void runSetpoint(Distance extension, double rollerOutput) {
        io.setExtensionPosition(extension);
        io.setRollerOutput(rollerOutput);
        extensionGoal = extension;
    }

    @AutoLogOutput
    public boolean isAtExtensionGoal() {
        return isAtPosition(extensionGoal);
    }

    public boolean isAtPosition(Distance position) {
        return inputs.extensionPosition.isNear(position, IntakeConstants.extensionTolerance);
    }

    public Command intakeEfficient() {
        var debouncer = new Debouncer(1.0, DebounceType.kFalling);
        return run(() -> runSetpoint(IntakeConstants.extendedPosition, 0.0))
                .until(() -> inputs.extensionPosition.gt(IntakeConstants.extendedRollerActivationPosition))
                .unless(() -> inputs.extensionPosition.gt(IntakeConstants.extendedRollerActivationPosition))
                .andThen(run(() -> {
                    // Run intake if robot is moving toward intake or spinning
                    var robotVelocity = RobotState.getInstance().getTargetRobotVelocity();
                    var rollerOutput = debouncer.calculate(robotVelocity.vxMetersPerSecond < -Constants.epsilon
                                    || !MathUtil.isNear(0, robotVelocity.omegaRadiansPerSecond, 0.1))
                            ? .95
                            : 0;

                    runSetpoint(extensionGoal, rollerOutput);
                }))
                .withName("Intake/IntakeEfficient");
    }

    public Command intake() {
        return run(() -> runSetpoint(IntakeConstants.extendedPosition, .95)).withName("Intake/Intake");
    }

    public Command reverse() {
        return run(() -> runSetpoint(IntakeConstants.extendedPosition, -.95)).withName("Intake/Reverse");
    }

    public Command stow() {
        return run(() -> runSetpoint(IntakeConstants.retractedPosition, 0)).withName("Intake/Stow");
    }

    public Command agitate() {
        return sequence(
                        run(() -> runSetpoint(IntakeConstants.partiallyExtendedPosition, .95))
                                .until(() -> isAtExtensionGoal()),
                        run(() -> runSetpoint(IntakeConstants.extendedPosition, .95))
                                .until(() -> isAtExtensionGoal())
                                .withTimeout(1.0))
                .repeatedly();
    }

    public Command stop() {
        return run(() -> {
                    io.stop();
                })
                .withName("Intake/Stop");
    }

    public Command zeroRoutine() {
        var debouncer = new Debouncer(0.5, DebounceType.kRising);
        return runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> io.setExtensionOutput(zeroRoutineOutput.get(), true)))
                .until(() -> debouncer.calculate(
                        inputs.extensionVelocity.isNear(MetersPerSecond.zero(), zeroVelocityTolerance.get())))
                .andThen(runOnce(() -> io.setExtensionEncoderPosition(IntakeConstants.maximumExtension)))
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .withName("Intake/ZeroRoutine");
    }

    public SysIdCommandSet extensionSysId() {
        var timeout = Seconds.of(10);
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(7).div(timeout),
                        Volts.of(2),
                        timeout,
                        state -> Logger.recordOutput("SysId/IntakeExtension", state.toString())),
                new SysIdRoutine.Mechanism(voltage -> io.setExtensionVoltage(voltage), null, this));

        var maxPosition = IntakeConstants.maximumExtension.minus(Inches.of(0.5));
        var minPosition = IntakeConstants.minimumExtension.plus(Inches.of(0.5));

        var debouncer = new Debouncer(1.0);
        Function<Distance, Command> goToStartingPosition = position -> runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> io.setExtensionPosition(position)))
                .until(() -> debouncer.calculate(
                        inputs.extensionPosition.isNear(position, IntakeConstants.extensionTolerance)));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .untilForward(() -> inputs.extensionPosition.gt(maxPosition))
                .untilReverse(() -> inputs.extensionPosition.lt(minPosition))
                .beforeStartingForward(() -> goToStartingPosition.apply(minPosition))
                .beforeStartingReverse(() -> goToStartingPosition.apply(maxPosition))
                .finallyDo(() -> io.stop())
                .build();
    }
}
