// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.apriltag.AprilTagCamera;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;

public class Vision extends SubsystemBase {
    private final AprilTagCamera[] cameras;
    private final RobotState robotState = RobotState.getInstance();

    public Vision(AprilTagIO[] tagIO) {
        this.cameras = Stream.of(tagIO).map(io -> new AprilTagCamera(io)).toArray(AprilTagCamera[]::new);
    }

    @Override
    public void periodic() {
        List<RobotState.VisionMeasurement> measurements = new ArrayList<>(cameras.length * 4);
        for (var camera : cameras) {
            camera.update(measurements);
        }

        robotState.addVisionMeasurements(measurements.toArray(RobotState.VisionMeasurement[]::new));
    }
}
