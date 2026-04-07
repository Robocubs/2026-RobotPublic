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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.characterization.SysIdCommandSet;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.CompositePowerLogger;
import frc.robot.util.PowerLogger;
import frc.robot.util.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Intake extends SubsystemBase {
    private static final LoggedTunableNumber zeroRoutineOutput =
            new LoggedTunableNumber("Intake/ZeroRoutineOutput", 0.4);
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private final PowerLogger leaderPowerLogger = new PowerLogger("IntakeLeader");
    private final PowerLogger followerPowerLogger = new PowerLogger("IntakeFollower");
    private final CompositePowerLogger powerLogger =
            new CompositePowerLogger("Intake", leaderPowerLogger, followerPowerLogger);

    private @AutoLogOutput Distance extensionGoal = IntakeConstants.minimumExtension;

    public Intake(IntakeIO io) {
        this.io = io;

        RobotState.getInstance().setHopperExtensionSupplier(() -> inputs.extension1Position);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        leaderPowerLogger.log(inputs.leaderStatorCurrent, inputs.leaderVoltage);
        followerPowerLogger.log(inputs.followerStatorCurrent, inputs.followerVoltage);
        powerLogger.log();
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
        return inputs.extension1Position.isNear(position, IntakeConstants.extensionTolerance)
                && inputs.extension2Position.isNear(position, IntakeConstants.extensionTolerance);
    }

    public Command defaultCommand() {
        var switchCheckDebouncer = new Debouncer(1.0);
        var zeroRoutine = zeroRoutine();
        var autoZero = sequence(
                Commands.runOnce(() -> switchCheckDebouncer.calculate(false)),
                Commands.waitSeconds(1.0),
                Commands.idle()
                        .until(() -> switchCheckDebouncer.calculate(!inputs.limitSwitch1 || !inputs.limitSwitch2)),
                Commands.runOnce(() -> CommandScheduler.getInstance().schedule(zeroRoutine)));
        return either(
                        parallel(intakeEfficient(), autoZero),
                        stop(),
                        () -> inputs.extension1Position.gt(IntakeConstants.mostlyExtendedPosition)
                                || inputs.extension2Position.gt(IntakeConstants.mostlyExtendedPosition))
                .withName("Intake/Default");
    }

    public Command intakeEfficient() {
        var debouncer = new Debouncer(1.0, DebounceType.kFalling);
        return run(() -> runSetpoint(IntakeConstants.extendedPosition, 0.0))
                .until(() -> inputs.extension1Position.gt(IntakeConstants.extendedRollerActivationPosition)
                        && inputs.extension2Position.gt(IntakeConstants.extendedRollerActivationPosition))
                .unless(() -> inputs.extension1Position.gt(IntakeConstants.extendedRollerActivationPosition)
                        && inputs.extension2Position.gt(IntakeConstants.extendedRollerActivationPosition))
                .andThen(run(() -> {
                    // Run intake if robot is moving toward intake or spinning
                    double rollerOutput;
                    var robotVelocity = RobotState.getInstance().getTargetRobotVelocity();
                    if (!debouncer.calculate(robotVelocity.vxMetersPerSecond < Constants.epsilon
                            || !MathUtil.isNear(0, robotVelocity.omegaRadiansPerSecond, 0.1))) {
                        rollerOutput = 0.0;
                    } else if (robotVelocity.vxMetersPerSecond > -DriveConstants.maxSpeed.in(MetersPerSecond) / 2) {
                        rollerOutput = 0.5;
                    } else {
                        var maxVelocity = DriveConstants.maxSpeed.in(MetersPerSecond);
                        rollerOutput = Math.abs(robotVelocity.vxMetersPerSecond) / maxVelocity;
                    }

                    runSetpoint(IntakeConstants.extendedPosition, rollerOutput);
                }))
                .withName("Intake/IntakeEfficient");
    }

    public Command intakeWithAutoUnjam() {
        var switchCheckDebouncer = new Debouncer(1.0);
        return Commands.repeatingSequence(
                        intake().until(() ->
                                switchCheckDebouncer.calculate(!inputs.limitSwitch1 || !inputs.limitSwitch2)),
                        zeroRoutine())
                .withName("Intake/IntakeWithAutoUnjam");
    }

    public Command intake() {
        return run(() -> runSetpoint(IntakeConstants.extendedPosition, 1.0)).withName("Intake/Intake");
    }

    public Command reverse() {
        return run(() -> runSetpoint(IntakeConstants.extendedPosition, -1.0)).withName("Intake/Reverse");
    }

    public Command stow() {
        return run(() -> runSetpoint(IntakeConstants.retractedPosition, 0)).withName("Intake/Stow");
    }

    public Command agitate() {
        return sequence(
                        run(() -> runSetpoint(IntakeConstants.partiallyExtendedPosition, 0.5))
                                .until(() -> isAtExtensionGoal())
                                .withTimeout(1.0),
                        run(() -> runSetpoint(IntakeConstants.extendedPosition, 0.5))
                                .until(() -> isAtExtensionGoal())
                                .withTimeout(1.0))
                .repeatedly()
                .withName("Intake/Agitate");
    }

    public Command stop() {
        return run(() -> {
                    io.stop();
                })
                .withName("Intake/Stop");
    }

    public Command zeroRoutine() {
        return run(() -> {
                    io.setExtensionOutput(
                            inputs.limitSwitch1 ? 0.0 : zeroRoutineOutput.get(),
                            inputs.limitSwitch2 ? 0.0 : zeroRoutineOutput.get(),
                            true);
                    io.setRollerOutput(
                            inputs.extension1Position.gt(IntakeConstants.extendedRollerActivationPosition)
                                            && inputs.extension2Position.gt(
                                                    IntakeConstants.extendedRollerActivationPosition)
                                    ? 0.9
                                    : 0.0);
                })
                .until(() -> inputs.limitSwitch1 && inputs.limitSwitch2)
                .withTimeout(1.5)
                .andThen(run(() -> io.setExtensionOutput(0.0)).withTimeout(0.1))
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
                        inputs.extension1Position.isNear(position, IntakeConstants.extensionTolerance)));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .untilForward(() -> inputs.extension1Position.gt(maxPosition))
                .untilReverse(() -> inputs.extension1Position.lt(minPosition))
                .beforeStartingForward(() -> goToStartingPosition.apply(minPosition))
                .beforeStartingReverse(() -> goToStartingPosition.apply(maxPosition))
                .finallyDo(() -> io.stop())
                .build();
    }
}
