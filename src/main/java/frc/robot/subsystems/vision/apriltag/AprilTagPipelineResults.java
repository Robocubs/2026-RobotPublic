// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.apriltag.proto.AprilTagPipelineResultsProto;

public class AprilTagPipelineResults implements ProtobufSerializable {
    public final AprilTagPipelineResult[] results;

    public AprilTagPipelineResults() {
        results = new AprilTagPipelineResult[] {};
    }

    public AprilTagPipelineResults(AprilTagPipelineResult[] results) {
        this.results = results;
    }

    public static final AprilTagPipelineResultsProto proto = new AprilTagPipelineResultsProto();
}
