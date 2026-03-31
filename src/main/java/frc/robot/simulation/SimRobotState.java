// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.flywheel.FlywheelConstants;
import frc.robot.subsystems.superstructure.indexer.IndexerConstants;
import frc.robot.subsystems.superstructure.kicker.KickerConstants;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.tuning.LoggedTunableBoolean;
import frc.robot.util.tuning.LoggedTunableNumber;
import frc.robot.util.tuning.LoggedTunableValue;
import lombok.Setter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.flywheelRadius;

public class SimRobotState {
    private static LoggedTunableBoolean useEfficiencyMode =
            new LoggedTunableBoolean("Simulation/UseEfficiencyMode", false);
    private static LoggedTunableNumber maxBallsPerSecond = new LoggedTunableNumber("Simulation/MaxBallsPerSecond", 8.0);

    public static final Time simLoopPeriod = Seconds.of(0.004);
    public static final Pose2d initialPose = new Pose2d(FieldConstants.center.getX(), 1.0, Rotation2d.kCW_90deg);

    private static final double fuelDiameterMeters = 0.15;
    private static final AngularVelocity minIndexerVelocityToShoot = RotationsPerSecond.of(0.1);
    private static final AngularVelocity minKickerVelocityToShoot = RotationsPerSecond.of(0.1);

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> poseBuffer =
            ConcurrentTimeInterpolatableBuffer.createBuffer(RobotState.lookbackTimeSeconds);
    private final List<Consumer<Pose2d>> poseResetConsumers = new ArrayList<>();

    private boolean hasInit = false;
    private Notifier simNotifier;
    private MapleSimArena arena;
    private SwerveDriveSimulation swerveDriveSim;
    private IntakeSimulation intakeSim;

    private @Setter Angle turretAngle = Radians.zero();
    private @Setter AngularVelocity turretVelocity = RadiansPerSecond.zero();
    private @Setter Angle hoodAngle = Radians.zero();
    private @Setter AngularVelocity flywheelVelocity = RadiansPerSecond.zero();
    private @Setter AngularVelocity kickerVelocity = RadiansPerSecond.zero();
    private @Setter AngularVelocity indexerVelocity = RadiansPerSecond.zero();

    private double lastShotOrIdleTimestamp = 0.0;

    public SimRobotState() {
        poseBuffer.addSample(Timer.getFPGATimestamp(), Pose2d.kZero);

        if (Constants.mapleSimEnabled) {
            arena = new MapleSimArena();
            arena.setEfficiencyMode(useEfficiencyMode.get());
            arena.resetFieldForAuto();

            SimulatedArena.overrideInstance(arena);
            SimulatedArena.overrideSimulationTimings(simLoopPeriod, 1);
        }
    }

    public synchronized void init(MapleSimSwerveDrivetrain drivetrainSim) {
        if (hasInit || !Constants.mapleSimEnabled) {
            return;
        }

        hasInit = true;

        swerveDriveSim = drivetrainSim.mapleSimDrive;

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                drivetrainSim.mapleSimDrive,
                RobotConstants.intakeWidth,
                RobotConstants.robotCenterToIntakeExtended.minus(RobotConstants.robotCenterToBackBumper),
                IntakeSide.BACK,
                RobotConstants.maxCapacity);
        intakeSim.setGamePiecesCount(8);

        simNotifier = new Notifier(() -> {
            try {
                SimulatedArena.getInstance().simulationPeriodic();
                drivetrainSim.update();
                addPoseMeasurement(Timer.getFPGATimestamp(), drivetrainSim.mapleSimDrive.getSimulatedDriveTrainPose());
            } catch (Exception e) {
                e.printStackTrace();
            }
        });
        simNotifier.startPeriodic(simLoopPeriod);
    }

    public void reset() {
        if (hasInit) {
            arena.setEfficiencyMode(useEfficiencyMode.get());
            arena.resetFieldForAuto();
            intakeSim.setGamePiecesCount(8);
        }
    }

    public void resetForAuto() {
        var autoPathStart = RobotState.getInstance().getAutoPathStart();
        if (!Pose2d.kZero.equals(autoPathStart)) {
            resetPose(autoPathStart);
            RobotState.getInstance().resetPose(autoPathStart);
        }
        if (hasInit) {
            arena.setEfficiencyMode(useEfficiencyMode.get());
            arena.resetFieldForAuto();
            intakeSim.setGamePiecesCount(8);
        }
    }

    public void update() {
        if (!hasInit) {
            return;
        }

        LoggedTunableValue.ifChanged(0, this::reset, useEfficiencyMode);

        var timestamp = Timer.getFPGATimestamp();
        var isShooting = kickerVelocity.gt(minKickerVelocityToShoot) && indexerVelocity.gt(minIndexerVelocityToShoot);
        var vBallInIndexer = indexerVelocity.in(RadiansPerSecond) * IndexerConstants.wheelRadius.in(Meters) / 2.0;
        var vBallInKicker =
                kickerVelocity.in(RadiansPerSecond) * KickerConstants.beltRadius.in(Meters) / 2.0; // accounts for ramp
        var secondsPerBall =
                Math.max(fuelDiameterMeters / Math.min(vBallInIndexer, vBallInKicker), 1.0 / maxBallsPerSecond.get());
        if (isShooting
                && timestamp - lastShotOrIdleTimestamp > secondsPerBall
                && intakeSim.obtainGamePieceFromIntake()) {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new RebuiltFuelOnFly(
                            swerveDriveSim
                                    .getSimulatedDriveTrainPose()
                                    .transformBy(TurretConstants.robotToTurret2d)
                                    .getTranslation(),
                            Translation2d.kZero, // Math is wrong, so just use turret pose for robot pose
                            calculateTurretFieldSpeeds(),
                            swerveDriveSim
                                    .getSimulatedDriveTrainPose()
                                    .getRotation()
                                    .plus(new Rotation2d(turretAngle)),
                            TurretConstants.robotToTurret
                                    .getMeasureZ()
                                    .plus(FlywheelConstants.turretToFlywheel.getMeasureZ())
                                    .plus(flywheelRadius
                                            .plus(Meters.of(fuelDiameterMeters / 2.0))
                                            .times(Math.cos(hoodAngle.in(Radians)))),
                            MetersPerSecond.of(
                                    (FlywheelConstants.flywheelRadius.in(Meters) * FlywheelConstants.gearReduction
                                                    + FlywheelConstants.rollerRadius.in(Meters)
                                                            * FlywheelConstants.rollerReduction)
                                            * flywheelVelocity.in(RadiansPerSecond)
                                            * 0.5),
                            Degrees.of(90).minus(hoodAngle)));
            lastShotOrIdleTimestamp = timestamp;
        } else if (!isShooting) {
            lastShotOrIdleTimestamp = timestamp;
        }

        var turretCenterPose =
                new Pose3d(swerveDriveSim.getSimulatedDriveTrainPose()).transformBy(TurretConstants.robotToTurret);
        var turretPose = new Pose3d(
                turretCenterPose.getTranslation(),
                turretCenterPose.getRotation().plus(new Rotation3d(0, 0, turretAngle.in(Radians))));

        Logger.recordOutput("Simulation/Pose", swerveDriveSim.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/TurretPose", turretPose);
        Logger.recordOutput("Simulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput("Simulation/FuelHeld", intakeSim.getGamePiecesAmount());
    }

    private ChassisSpeeds calculateTurretFieldSpeeds() {
        var robotSpeeds = swerveDriveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        var robotHeading = swerveDriveSim.getSimulatedDriveTrainPose().getRotation();

        var turretOffsetField = TurretConstants.robotToTurret2d.getTranslation().rotateBy(robotHeading);

        var omega = robotSpeeds.omegaRadiansPerSecond;
        var vx = robotSpeeds.vxMetersPerSecond - omega * turretOffsetField.getY();
        var vy = robotSpeeds.vyMetersPerSecond + omega * turretOffsetField.getX();

        return new ChassisSpeeds(vx, vy, omega + turretVelocity.in(RadiansPerSecond));
    }

    public void addPoseMeasurement(double timestamp, Pose2d pose) {
        poseBuffer.addSample(timestamp, pose);
    }

    public void addPoseResetConsumer(Consumer<Pose2d> consumer) {
        poseResetConsumers.add(consumer);
    }

    public Pose2d getPose() {
        return poseBuffer.getSample(Timer.getFPGATimestamp()).orElse(Pose2d.kZero);
    }

    public Pose2d getPoseAt(double timestamp) {
        return poseBuffer.getSample(timestamp).orElse(Pose2d.kZero);
    }

    public void resetPose(Pose2d pose) {
        if (swerveDriveSim != null) {
            swerveDriveSim.setSimulationWorldPose(pose);
            Timer.delay(Seconds.of(0.05));
        }

        poseBuffer.clear();
        poseBuffer.addSample(Timer.getFPGATimestamp(), pose);

        poseResetConsumers.forEach(consumer -> consumer.accept(pose));
    }

    public void resetRotation(Rotation2d rotation) {
        if (swerveDriveSim != null) {
            var pose = swerveDriveSim.getSimulatedDriveTrainPose();
            resetPose(new Pose2d(pose.getTranslation(), rotation));
        }
    }

    public void updateIntakeSim(boolean isExtendedAndRunning) {
        if (intakeSim == null) {
            return;
        }

        if (isExtendedAndRunning) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }
    }
}
