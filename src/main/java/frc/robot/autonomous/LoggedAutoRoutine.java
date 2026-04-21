// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.commands.logging.LoggedCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import frc.robot.util.PathBuilder;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class LoggedAutoRoutine {
    private final AutoFactory factory;
    private final AutoRoutine routine;
    private final String name;
    private final RobotState robotState = RobotState.getInstance();
    private final Drive drive;
    private final List<Command> commands = new ArrayList<>();
    private final PathBuilder pathBuilder = new PathBuilder();
    private final SubsystemScheduler<Superstructure> superstructure;
    private final SubsystemScheduler<Intake> intake;

    private boolean bound = false;
    private static final Pose2d stationPose = new Pose2d(0.688, 0.643, Rotation2d.kZero);

    public LoggedAutoRoutine(
            String name,
            AutoFactory factory,
            Drive drive,
            SubsystemScheduler<Superstructure> superstructure,
            SubsystemScheduler<Intake> intake) {
        this.factory = factory;
        this.routine = factory.newRoutine(name);
        this.name = name;
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;
    }

    public AutoRoutine build() {
        if (!bound) {
            routine.active()
                    .onTrue(sequence(
                            print("Starting " + name),
                            deadline(
                                    LoggedCommands.loggedSequence(commands.toArray(Command[]::new)),
                                    superstructure.cmd(),
                                    intake.cmd()),
                            print("Finished " + name)));
        }

        var path = GeometryUtil.autoFlip(pathBuilder.build());
        Logger.recordOutput("Auto/Path", path);
        robotState.setPath(path);

        return routine;
    }

    public LoggedAutoRoutine requireStartingPose(Pose2d pose) {
        if (Constants.currentMode == Constants.Mode.SIM) {
            this.resetPose(pose);
        }

        commands.add(print("Starting pose too far, doing nothing")
                .andThen(idle())
                .unless(() ->
                        robotState.getPose().getTranslation().getDistance(GeometryUtil.autoFlip(pose.getTranslation()))
                                < 1.0));
        pathBuilder.add(pose);

        return this;
    }

    public LoggedAutoRoutine requireStartingPose(String pathName) {
        var trajectory = routine.trajectory(pathName);
        if (trajectory == null) {
            commands.add(runOnce(() -> System.out.println("Trajectory not found, doing nothing"))
                    .andThen(idle()));
            return this;
        }

        var startingPoseOptional = trajectory.getRawTrajectory().getInitialPose(false);
        if (startingPoseOptional.isEmpty()) {
            commands.add(runOnce(() -> System.out.println("Starting pose not found, doing nothing"))
                    .andThen(idle()));
            return this;
        }

        requireStartingPose(startingPoseOptional.get());
        return this;
    }

    public LoggedAutoRoutine run(Command command) {
        return this;
    }

    public LoggedAutoRoutine waitSeconds(double seconds) {
        commands.add(Commands.waitSeconds(seconds));
        return this;
    }

    public LoggedAutoRoutine velocitySeconds(ChassisSpeeds chassisSpeeds, double seconds) {
        commands.add(drive.withRobotSpeeds(() -> chassisSpeeds).withTimeout(seconds));
        return this;
    }

    public LoggedAutoRoutine rotateTo(Rotation2d rotation) {
        commands.add(drive.toPose(
                () -> {
                    var pose = robotState.getPose();
                    return new Pose2d(pose.getX(), pose.getY(), rotation);
                },
                true,
                false));
        return this;
    }

    public LoggedAutoRoutine resetRotation(Rotation2d rotation) {
        commands.add(runOnce(() -> {
            robotState.resetHeading(GeometryUtil.autoFlip(rotation));
        }));

        return this;
    }

    public Pose2d getInitialPose(String pathName) {
        return routine.trajectory(pathName)
                .getRawTrajectory()
                .getInitialPose(false)
                .get();
    }

    public Pose2d getFinalPose(String pathName, int splitIndex) {
        return routine.trajectory(pathName, splitIndex)
                .getRawTrajectory()
                .getFinalPose(false)
                .get();
    }

    public LoggedAutoRoutine resetPose(Pose2d pose) {
        commands.add(factory.resetOdometry(() -> Optional.of(GeometryUtil.autoFlip(pose))));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine resetPose(String pathName) {
        commands.add(factory.resetOdometry(pathName));
        routine.trajectory(pathName).getRawTrajectory().getInitialPose(false).ifPresent(pathBuilder::add);
        return this;
    }

    public LoggedAutoRoutine followPath(String pathName) {
        var path = routine.trajectory(pathName);
        commands.add(path.cmd());
        pathBuilder.add(path.getRawTrajectory().getPoses());
        return this;
    }

    public LoggedAutoRoutine followPath(String pathName, int splitIndex) {
        var path = routine.trajectory(pathName, splitIndex);
        commands.add(path.cmd());
        pathBuilder.add(path.getRawTrajectory().getPoses());
        return this;
    }

    public LoggedAutoRoutine driveToPose(Pose2d pose) {
        driveToPose(pose, false);
        return this;
    }

    public LoggedAutoRoutine driveToPose(Pose2d pose, boolean slowMode) {
        commands.add(drive.toPose(() -> GeometryUtil.autoFlip(pose), true, slowMode));
        pathBuilder.add(pose);
        return this;
    }

    public LoggedAutoRoutine driveToInitialPose(String pathName) {
        driveToInitialPose(pathName, false);
        return this;
    }

    public LoggedAutoRoutine driveToInitialPose(String pathName, boolean slowMode) {
        var initialPose = getInitialPose(pathName);
        var initialPoseFlipped = GeometryUtil.autoFlip(initialPose);
        commands.add(drive.toPose(() -> initialPoseFlipped, true, slowMode)
                .until(() ->
                        robotState.getPose().getTranslation().getDistance(initialPoseFlipped.getTranslation()) <= 0.2));
        pathBuilder.add(initialPose);
        return this;
    }

    public LoggedAutoRoutine driveToFinalPose(String pathName, int splitIndex, boolean finishAtGoal, boolean slowMode) {
        var initialPose = getFinalPose(pathName, splitIndex);
        var initialPoseFlipped = GeometryUtil.autoFlip(initialPose);
        commands.add(drive.toPose(() -> initialPoseFlipped, finishAtGoal, slowMode));
        pathBuilder.add(initialPose);
        return this;
    }

    public LoggedAutoRoutine driveToStation(double seconds) {
        var stationPoseFlipped = GeometryUtil.autoFlip(stationPose);
        commands.add(drive.toPose(() -> stationPoseFlipped, true, true)
                .until(() ->
                        robotState.getPose().getTranslation().getDistance(stationPoseFlipped.getTranslation()) <= 0.2));
        commands.add(drive.toPose(() -> stationPoseFlipped, false, true).withTimeout(Seconds.of(seconds)));
        pathBuilder.add(stationPose);
        return this;
    }

    public LoggedAutoRoutine setSuperstructureState(SuperstructureState state) {
        commands.add(superstructure.schedule(s -> s.trackingHubForPose()));
        return this;
    }

    public LoggedAutoRoutine followPathAndShoot(String pathName, int splitIndex, double shootSeconds) {
        var path = routine.trajectory(pathName, splitIndex);
        commands.add(path.cmd());
        pathBuilder.add(path.getRawTrajectory().getPoses());

        commands.add(deadline(
                // Hold the robot still while shooting for shootSeconds, then the sequence continues
                Commands.waitSeconds(shootSeconds),
                drive.withRobotSpeeds(() -> new ChassisSpeeds()),
                superstructure.schedule(s -> s.shoot())));
        commands.add(superstructure.schedule(s -> s.trackingHubForPose()));
        return this;
    }

    public LoggedAutoRoutine pointModulesUntil(Rotation2d target, BooleanSupplier condition) {
        commands.add(drive.pointModules(() -> target).until(condition).withName("PointModulesAndWaitForCondition"));
        return this;
    }

    public LoggedAutoRoutine pointModulesUntil(Translation2d target, BooleanSupplier condition) {
        commands.add(drive.pointModulesAt(() -> GeometryUtil.autoFlip(target))
                .until(condition)
                .withName("PointModulesAndWaitForCondition"));
        return this;
    }

    public LoggedAutoRoutine pointModulesWaitSeconds(Translation2d target, double seconds) {
        commands.add(drive.pointModulesAt(() -> GeometryUtil.autoFlip(target))
                .withTimeout(seconds)
                .withName("PointModulesAndWait"));
        return this;
    }

    public LoggedAutoRoutine intake() {
        commands.add(intake.schedule(i -> i.intakeWithAutoUnjam()));
        return this;
    }

    public LoggedAutoRoutine intakeEfficient() {
        commands.add(intake.schedule(i -> i.intakeEfficient()));
        return this;
    }

    public LoggedAutoRoutine stowIntake() {
        commands.add(intake.schedule(i -> i.stow()));
        return this;
    }

    public LoggedAutoRoutine agitateIntake() {
        commands.add(intake.schedule(i -> i.agitate()));
        return this;
    }

    public LoggedAutoRoutine shoot() {
        commands.add(superstructure.schedule(s -> s.shoot()));
        return this;
    }

    public LoggedAutoRoutine shoot(double shootSeconds) {
        commands.add(deadline(
                // Hold the robot still while shooting for shootSeconds, then the sequence continues
                Commands.waitSeconds(shootSeconds),
                drive.withRobotSpeeds(() -> new ChassisSpeeds()),
                superstructure.schedule(s -> s.shoot())));
        return this;
    }

    public LoggedAutoRoutine shootAndOscillate(double seconds) {
        commands.add(deadline(
                Commands.waitSeconds(seconds),
                DriveCommands.oscillate(drive),
                superstructure.schedule(s -> s.shoot())));
        return this;
    }
}
