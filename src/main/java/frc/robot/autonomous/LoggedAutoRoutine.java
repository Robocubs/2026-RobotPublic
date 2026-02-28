// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
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
import frc.robot.commands.logging.LoggedCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeometryUtil;
import frc.robot.util.PathBuilder;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class LoggedAutoRoutine {
    private final AutoFactory factory;
    private final AutoRoutine routine;
    private final String name;
    private final RobotState robotState = RobotState.getInstance();
    private final Drive drive;
    private final List<Command> commands = new ArrayList<>();
    private final PathBuilder pathBuilder = new PathBuilder();

    private boolean bound = false;

    public LoggedAutoRoutine(String name, AutoFactory factory, Drive drive) {
        this.factory = factory;
        this.routine = factory.newRoutine(name);
        this.name = name;
        this.drive = drive;
    }

    public AutoRoutine build() {
        if (!bound) {
            routine.active()
                    .onTrue(sequence(
                            print("Starting " + name),
                            LoggedCommands.loggedSequence(commands.toArray(Command[]::new)),
                            print("Finished " + name)));
        }

        var path = GeometryUtil.autoFlip(pathBuilder.build());
        Logger.recordOutput("Auto/Path", path);
        robotState.setPath(path);

        return routine;
    }

    public LoggedAutoRoutine requireStartingPose(String pathName) {
        if (Constants.currentMode == Constants.Mode.SIM) {
            return this.resetPose(pathName);
        }

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

        var startingPose = startingPoseOptional.get();
        commands.add(print("Starting pose too far, doing nothing")
                .andThen(idle())
                .unless(() -> robotState
                                .getPose()
                                .getTranslation()
                                .getDistance(GeometryUtil.autoFlip(startingPose.getTranslation()))
                        < 1.0));
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
}
