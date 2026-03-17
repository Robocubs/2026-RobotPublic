// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.simulation.SimRobotState;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagIOSim extends AprilTagIOHardware {
    public static VisionSystemSim visionSim;

    private final PhotonCameraSim cameraSim;
    private final SimRobotState simRobotState;

    public AprilTagIOSim(AprilTagConfig config, SimRobotState simRobotState) {
        super(config);

        this.simRobotState = simRobotState;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.fieldLayout);
            simRobotState.addPoseResetConsumer(visionSim::resetRobotPose);
        }

        cameraSim = new PhotonCameraSim(
                camera, new SimCameraProperties().setCalibration(1280, 800, Rotation2d.fromDegrees(97.65)));
        cameraSim.setMaxSightRange(4.5);
        visionSim.addCamera(cameraSim, config.robotToCamera.apply(0.0));
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        visionSim.update(simRobotState.getPose());
        visionSim.adjustCamera(cameraSim, config.robotToCamera.apply(Timer.getTimestamp()));

        super.updateInputs(inputs);
    }
}
