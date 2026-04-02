// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.commands.ContinuousSwitchCommand;
import frc.robot.commands.characterization.SysIdCommandSet;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.flywheel.FlywheelIO;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.hood.HoodConstants;
import frc.robot.subsystems.superstructure.hood.HoodIO;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIO;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.kicker.KickerIO;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import frc.robot.subsystems.superstructure.turret.TurretIO;
import frc.robot.util.BidirectionalDebouncer;
import frc.robot.util.TrapezoidProfileController;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;

public class Superstructure extends SubsystemBase {
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> flywheelVelocity =
            new LoggedTunableMeasure<>("Superstructure/FlywheelVelocity", RadiansPerSecond.of(285));
    private static final LoggedTunableMeasure<AngleUnit, Angle> tunedHoodAngle =
            new LoggedTunableMeasure<>("Superstructure/HoodAngle", Radians.of(0.464));
    private static final LoggedTunableMeasure<AngleUnit, Angle> tunedTurretAngle =
            new LoggedTunableMeasure<>("Superstructure/TurretAngle", Radians.of(0));

    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> indexerShootingVelocity =
            new LoggedTunableMeasure<>("Superstructure/IndexerShootingVelocity", RadiansPerSecond.of(40));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> kickerShootingVelocity =
            new LoggedTunableMeasure<>("Superstructure/KickerShootingVelocity", RadiansPerSecond.of(120));

    private static final LoggedTunableMeasure<TimeUnit, Time> timeToShoot =
            new LoggedTunableMeasure<>("Superstructure/TimeToShoot", Seconds.of(2));
    private static final LoggedTunableMeasure<TimeUnit, Time> timeToScan =
            new LoggedTunableMeasure<>("Superstructure/TimeToScan", Seconds.of(5));
    private static final LoggedTunableNumber requiredScanMeasurementCount =
            new LoggedTunableNumber("Superstructure/RequiredScanMeasurementCount", 7);

    private static final BidirectionalDebouncer indexingDebouncer = new BidirectionalDebouncer(0.25, 3.0, true);

    private final RobotState robotState = RobotState.getInstance();
    private final Flywheel flywheel;
    private final Hood hood;
    private final Turret turret;
    private final Indexer indexer;
    private final Kicker kicker;

    private final SuperstructureSetpointCalculator superstructureSetpointCalculator =
            new SuperstructureSetpointCalculator();
    private final TrapezoidProfileController scanController = new TrapezoidProfileController(Math.PI / 2, 10.0);

    private @AutoLogOutput @Getter SuperstructureState state = SuperstructureState.STOP;
    private int lastScanVisionMeasurementCount = 0;

    public static enum SuperstructureState {
        STOP,
        TRACKING_HUB_FOR_POSE,
        TRACKING_HUB_FOR_POSE_IDLE,
        SHOOTING_HUB,
        TRACKING_HUB_BEFORE_SHOOTING,
        TRACKING_BEFORE_FEEDING,
        FEEDING,
        TUNING,
        TUNING_WARMING_UP,
        SCAN,
        TURRET_SYSID,
        HOOD_SYSID,
        FLYWHEEL_SYSID,
        INDEXER_SYSID,
        KICKER_SYSID,
    }

    public static record SuperstructureSetpoint(
            AngularVelocity flywheelVelocity, Angle hoodAngle, Angle turretAngle, AngularVelocity turretVelocity) {}

    public Superstructure(
            FlywheelIO flywheelIO, HoodIO hoodIO, TurretIO turretIO, IndexerIO indexerIO, KickerIO kickerIO) {
        this.flywheel = new Flywheel(flywheelIO);
        this.hood = new Hood(hoodIO);
        this.turret = new Turret(turretIO);
        this.indexer = new Indexer(indexerIO);
        this.kicker = new Kicker(kickerIO);
    }

    @Override
    public void periodic() {
        flywheel.periodic();
        hood.periodic();
        turret.periodic();
        indexer.periodic();
        kicker.periodic();

        robotState.addSuperstructureMeasurement(turret.getPosition(), turret.getVelocity());
    }

    public void applyState(SuperstructureState state) {
        this.state = state;

        if (state != SuperstructureState.TRACKING_BEFORE_FEEDING && state != SuperstructureState.FEEDING) {
            superstructureSetpointCalculator.resetPassingTarget();
        }

        switch (state) {
            case TRACKING_HUB_FOR_POSE -> {
                var superstructureSetpoint = superstructureSetpointCalculator.getSetpointForTrackingPose();
                flywheel.setVelocity(superstructureSetpoint.flywheelVelocity);
                hood.setAngle(superstructureSetpoint.hoodAngle);
                turret.setAngle(superstructureSetpoint.turretAngle, superstructureSetpoint.turretVelocity);
                indexer.stop();
                kicker.stop();
            }
            case TRACKING_HUB_FOR_POSE_IDLE -> {
                var superstructureSetpoint = superstructureSetpointCalculator.getSetpointForTrackingPose();
                flywheel.setVelocity(RadiansPerSecond.zero());
                hood.setAngle(superstructureSetpoint.hoodAngle);
                turret.setAngle(superstructureSetpoint.turretAngle, superstructureSetpoint.turretVelocity);
                indexer.stop();
                kicker.stop();
            }
            case SHOOTING_HUB -> {
                var shouldIndex = indexingDebouncer.calculate(!indexingDebouncer.getBaseline());
                var superstructureSetpoint = superstructureSetpointCalculator.getSetpointForShooting();
                flywheel.setVelocity(superstructureSetpoint.flywheelVelocity);
                hood.setAngle(superstructureSetpoint.hoodAngle);
                turret.setAngle(superstructureSetpoint.turretAngle, superstructureSetpoint.turretVelocity);
                if (turret.isWrapping()) {
                    indexer.stop();
                    kicker.stop();
                } else if (shouldIndex) {
                    indexer.setVelocity(indexerShootingVelocity.get());
                    kicker.setVelocity(kickerShootingVelocity.get());
                } else {
                    indexer.setVelocity(indexerShootingVelocity.get().times(-1.0));
                    kicker.setVelocity(kickerShootingVelocity.get());
                }
            }
            case TRACKING_HUB_BEFORE_SHOOTING -> {
                var superstructureSetpoint = superstructureSetpointCalculator.getSetpointForShooting();
                flywheel.setVelocity(superstructureSetpoint.flywheelVelocity);
                hood.setAngle(superstructureSetpoint.hoodAngle);
                turret.setAngle(superstructureSetpoint.turretAngle, superstructureSetpoint.turretVelocity);
                indexer.stop();
                kicker.stop();
            }
            case TRACKING_BEFORE_FEEDING -> {
                var superstructureSetpoint = superstructureSetpointCalculator.getSetpointForPassing();
                flywheel.setVelocity(superstructureSetpoint.flywheelVelocity);
                hood.setAngle(superstructureSetpoint.hoodAngle);
                turret.setAngle(superstructureSetpoint.turretAngle, superstructureSetpoint.turretVelocity);
                indexer.stop();
                kicker.stop();
            }
            case FEEDING -> {
                var shouldIndex = indexingDebouncer.calculate(!indexingDebouncer.getBaseline());
                var superstructureSetpoint = superstructureSetpointCalculator.getSetpointForPassing();
                flywheel.setVelocity(superstructureSetpoint.flywheelVelocity);
                hood.setAngle(superstructureSetpoint.hoodAngle);
                turret.setAngle(superstructureSetpoint.turretAngle, superstructureSetpoint.turretVelocity);
                if (turret.isWrapping()) {
                    indexer.stop();
                    kicker.stop();
                } else if (shouldIndex) {
                    indexer.setVelocity(indexerShootingVelocity.get());
                    kicker.setVelocity(kickerShootingVelocity.get());
                } else {
                    indexer.setVelocity(indexerShootingVelocity.get().times(-1.0));
                    kicker.setVelocity(kickerShootingVelocity.get());
                }
            }
            case SCAN -> {
                var turretSetpoint =
                        scanController.calculate(turret.getPosition().in(Radians));
                flywheel.stop();
                hood.stop();
                turret.setAngleUnoptimized(
                        Radians.of(turretSetpoint.position), RadiansPerSecond.of(turretSetpoint.velocity));
                indexer.stop();
                kicker.stop();
            }
            case TUNING -> {
                flywheel.setVelocity(flywheelVelocity.get());
                hood.setAngle(tunedHoodAngle.get());
                turret.setAngle(tunedTurretAngle.get());
                indexer.setVelocity(indexerShootingVelocity.get());
                kicker.setVelocity(kickerShootingVelocity.get());
            }
            case TUNING_WARMING_UP -> {
                flywheel.setVelocity(flywheelVelocity.get());
                hood.setAngle(tunedHoodAngle.get());
                turret.setAngle(tunedTurretAngle.get());
                indexer.setVelocity(RadiansPerSecond.zero());
                kicker.setVelocity(RadiansPerSecond.zero());
            }
            case FLYWHEEL_SYSID -> {
                hood.stop();
                turret.stop();
                indexer.stop();
                kicker.stop();
            }
            case HOOD_SYSID -> {
                flywheel.stop();
                turret.stop();
                indexer.stop();
                kicker.stop();
            }
            case TURRET_SYSID -> {
                flywheel.stop();
                hood.stop();
                indexer.stop();
                kicker.stop();
            }
            case INDEXER_SYSID -> {
                flywheel.stop();
                hood.stop();
                turret.stop();
                kicker.stop();
            }
            case KICKER_SYSID -> {
                flywheel.stop();
                hood.stop();
                turret.stop();
                indexer.stop();
            }
            default -> {
                flywheel.stop();
                hood.stop();
                turret.stop();
                indexer.stop();
                kicker.stop();
            }
        }
    }

    public Command defaultCommand() {
        return new ContinuousSwitchCommand(run(() -> applyState(SuperstructureState.TRACKING_HUB_FOR_POSE)))
                .withOption(
                        () -> Timer.getTimestamp() - robotState.getLastVisionMeasurementTimestamp()
                                        > timeToScan.get().in(Seconds)
                                || robotState.getTotalVisionMeasurementsCount() - lastScanVisionMeasurementCount
                                        < requiredScanMeasurementCount.get(),
                        runOnce(() -> lastScanVisionMeasurementCount = robotState.getTotalVisionMeasurementsCount())
                                .andThen(scan()))
                .withOption(
                        () -> !robotState.inAllianceZone() && !robotState.onAllianceBump(),
                        run(() -> applyState(SuperstructureState.TRACKING_HUB_FOR_POSE_IDLE)))
                .withName("Superstructure/Default");
    }

    public Command stop() {
        return run(() -> applyState(SuperstructureState.STOP)).withName("Superstructure/Stop");
    }

    public Command scan() {
        return runOnce(() -> scanController.reset(
                        turret.getPosition().in(Radians), turret.getVelocity().in(RadiansPerSecond)))
                .andThen(repeatingSequence(
                        runOnce(() -> scanController.withGoal(TurretConstants.maxAngle.in(Radians))),
                        run(() -> applyState(SuperstructureState.SCAN))
                                .until(() -> turret.isAtAngle(TurretConstants.maxAngle)),
                        runOnce(() -> scanController.withGoal(TurretConstants.minAngle.in(Radians))),
                        run(() -> applyState(SuperstructureState.SCAN))
                                .until(() -> turret.isAtAngle(TurretConstants.minAngle))))
                .withName("Superstructure/Scan");
    }

    public Command shoot() {
        return run(() -> applyState(SuperstructureState.TRACKING_HUB_BEFORE_SHOOTING))
                .until(() -> flywheel.isAtGoal()
                        && hood.isAtGoal()
                        && turret.isAtGoal()
                        && robotState.getMatchState().getTimeToHubActive().lt(timeToShoot.get())
                        && robotState.inAllianceZone())
                .andThen(runOnce(() -> indexingDebouncer.reset(true)))
                .andThen(run(() -> {
                            if (robotState.turretIsUnderTower()) {
                                applyState(SuperstructureState.TRACKING_HUB_BEFORE_SHOOTING);
                            } else {
                                applyState(SuperstructureState.SHOOTING_HUB);
                            }
                        })
                        .until(() ->
                                robotState.getMatchState().getTimeToHubActive().gt(timeToShoot.get())))
                .withName("Superstructure/Shoot");
    }

    public Command shootWithTuning() {
        return run(() -> applyState(SuperstructureState.TUNING_WARMING_UP))
                .until(() -> flywheel.isAtGoal() && hood.isAtGoal() && turret.isAtGoal())
                .andThen(run(() -> applyState(SuperstructureState.TUNING))
                        .until(() ->
                                robotState.getMatchState().getTimeToHubActive().gt(timeToShoot.get())))
                .withName("Superstructure/Tune");
    }

    public Command feed() {
        return run(() -> applyState(SuperstructureState.TRACKING_BEFORE_FEEDING))
                .until(() -> flywheel.isAtGoal() && hood.isAtGoal() && turret.isAtGoal())
                .andThen(runOnce(() -> indexingDebouncer.reset(true)))
                .andThen(run(() -> {
                    if (superstructureSetpointCalculator.isPassingBlocked()) {
                        applyState(SuperstructureState.TRACKING_BEFORE_FEEDING);
                    } else {
                        applyState(SuperstructureState.FEEDING);
                    }
                }))
                .withName("Superstructure/Feed");
    }

    public Command trackingHubForPose() {
        return run(() -> applyState(SuperstructureState.TRACKING_HUB_FOR_POSE))
                .withName("Superstructure/TrackHubForPose");
    }

    public Command zero() {
        return Commands.runOnce(() -> {
                    turret.resetPosition();
                })
                .ignoringDisable(true)
                .withName("Superstructure/Zero");
    }

    public SysIdCommandSet flywheelSysId() {
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, state -> Logger.recordOutput("SysId/Flywheel", state.toString())),
                new SysIdRoutine.Mechanism(voltage -> flywheel.setVoltage(voltage), null, this));

        var debouncer = new Debouncer(1.0);
        Supplier<Command> stopShooter = () -> runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> flywheel.stop()))
                .until(() -> debouncer.calculate(flywheel.isAtVelocity(RadiansPerSecond.zero())));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .beforeStarting(stopShooter)
                .alongWith(() -> Commands.run(() -> applyState(SuperstructureState.FLYWHEEL_SYSID)))
                .finallyDo(() -> applyState(SuperstructureState.STOP))
                .build();
    }

    public SysIdCommandSet hoodSysId() {
        var timeout = Seconds.of(10);
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.95).div(timeout),
                        Volts.of(0.7),
                        timeout,
                        state -> Logger.recordOutput("SysId/Hood", state.toString())),
                new SysIdRoutine.Mechanism(null, null, this));

        var maxAngle = HoodConstants.maxAngle.minus(Degrees.of(5));
        var minAngle = HoodConstants.minAngle.plus(Degrees.of(5));

        var debouncer = new Debouncer(1.0);
        Function<Angle, Command> goToStartingAngle = angle -> runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> hood.setAngle(angle)))
                .until(() -> debouncer.calculate(hood.isAtGoal()));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .untilForward(() -> hood.getAngle().gt(maxAngle))
                .untilReverse(() -> hood.getAngle().lt(minAngle))
                .beforeStartingForward(() -> goToStartingAngle.apply(minAngle))
                .beforeStartingReverse(() -> goToStartingAngle.apply(maxAngle))
                .alongWith(() -> Commands.run(() -> applyState(SuperstructureState.HOOD_SYSID)))
                .finallyDo(() -> applyState(SuperstructureState.STOP))
                .build();
    }

    public SysIdCommandSet turretSysId() {
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, Volts.of(4), null, state -> Logger.recordOutput("SysId/Turret", state.toString())),
                new SysIdRoutine.Mechanism(turret::setVoltage, null, this));

        var maxAngle = TurretConstants.maxAngle.minus(Degrees.of(20));
        var minAngle = TurretConstants.minAngle.plus(Degrees.of(20));

        var debouncer = new Debouncer(1.0);
        Function<Angle, Command> goToStartingAngle = angle -> runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> turret.setAngleUnoptimized(angle)))
                .until(() -> debouncer.calculate(turret.isAtGoal()));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .untilForward(() -> turret.getPosition().gt(maxAngle))
                .untilReverse(() -> turret.getPosition().lt(minAngle))
                .beforeStartingForward(() -> goToStartingAngle.apply(minAngle))
                .beforeStartingReverse(() -> goToStartingAngle.apply(maxAngle))
                .alongWith(() -> Commands.run(() -> applyState(SuperstructureState.TURRET_SYSID)))
                .finallyDo(() -> applyState(SuperstructureState.STOP))
                .build();
    }

    public SysIdCommandSet indexerSysId() {
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, Volts.of(4), null, state -> Logger.recordOutput("SysId/Indexer", state.toString())),
                new SysIdRoutine.Mechanism(indexer::setVoltage, null, this));

        var debouncer = new Debouncer(1.0);
        Supplier<Command> stopIndexer = () -> runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> indexer.stop()))
                .until(() -> debouncer.calculate(indexer.isAtVelocity(RadiansPerSecond.zero())));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .alongWith(() -> Commands.run(() -> applyState(SuperstructureState.INDEXER_SYSID)))
                .beforeStarting(stopIndexer)
                .finallyDo(() -> applyState(SuperstructureState.STOP))
                .build();
    }

    public SysIdCommandSet kickerSysId() {
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, Volts.of(4), null, state -> Logger.recordOutput("SysId/Kicker", state.toString())),
                new SysIdRoutine.Mechanism(kicker::setVoltage, null, this));

        var debouncer = new Debouncer(1.0);
        Supplier<Command> stopKicker = () -> runOnce(() -> debouncer.calculate(false))
                .andThen(run(() -> kicker.stop()))
                .until(() -> debouncer.calculate(kicker.isAtVelocity(RadiansPerSecond.zero())));

        return SysIdCommandSet.builder()
                .sysIdRoutine(routine)
                .alongWith(() -> Commands.run(() -> applyState(SuperstructureState.KICKER_SYSID)))
                .beforeStarting(stopKicker)
                .finallyDo(() -> applyState(SuperstructureState.STOP))
                .build();
    }
}
