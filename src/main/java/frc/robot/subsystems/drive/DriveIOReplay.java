// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.simulation.ReplayRobotState;

public class DriveIOReplay implements DriveIO {
    @Override
    public void resetPose(Pose2d pose2d) {
        ReplayRobotState.getInstance().resetPose(pose2d);
    }

    @Override
    public void resetRotation(Rotation2d rotation2d) {
        ReplayRobotState.getInstance().resetRotation(rotation2d);
    }

    @Override
    public void addVisionMeasurements(VisionMeasurement[] measurements) {
        ReplayRobotState.getInstance().addVisionMeasurements(measurements);
    }
}
