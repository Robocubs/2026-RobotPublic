// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import java.util.stream.Stream;

import com.google.errorprone.annotations.concurrent.LazyInit;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIO.SwerveDriveInputs;
import lombok.Getter;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = lombok.AccessLevel.PRIVATE)
public class ReplayRobotState {
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            Rotation2d.kZero,
            Stream.generate(SwerveModulePosition::new).limit(4).toArray(SwerveModulePosition[]::new),
            Pose2d.kZero,
            DriveConstants.odometryStdDevs,
            DriveConstants.visionStdDevs);

    private static @LazyInit @Getter ReplayRobotState instance = new ReplayRobotState();

    public SwerveDriveInputs[] update(SwerveDriveInputs[] inputs) {
        return Stream.of(inputs)
                .map(input -> {
                    poseEstimator.updateWithTime(input.timestamp, input.rawHeading, input.modulePositions);
                    return new SwerveDriveInputs(
                            poseEstimator.getEstimatedPosition(),
                            input.speeds,
                            input.moduleStates,
                            input.moduleTargets,
                            input.modulePositions,
                            input.rawHeading,
                            input.timestamp,
                            input.odometryPeriod,
                            input.successfulDaqs,
                            input.failedDaqs);
                })
                .toArray(SwerveDriveInputs[]::new);
    }

    public void addVisionMeasurements(VisionMeasurement[] measurements) {
        for (var measurement : measurements) {
            poseEstimator.addVisionMeasurement(
                    measurement.pose.toPose2d(), measurement.timestampSeconds, measurement.stdDevs);
        }
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void resetRotation(Rotation2d rotation) {
        poseEstimator.resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
    }
}
