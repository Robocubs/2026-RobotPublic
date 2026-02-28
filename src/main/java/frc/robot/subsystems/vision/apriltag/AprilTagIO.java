// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {
    @AutoLog
    public class AprilTagIOInputs {
        public boolean isConnected;
        public AprilTagPipelineResults pipelineResults = new AprilTagPipelineResults();
        public int fps;
        public int temperature;
        public int latency;
    }

    public default void updateInputs(AprilTagIOInputs inputs) {}

    public AprilTagConfig getConfig();
}
