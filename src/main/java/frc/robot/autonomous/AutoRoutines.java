// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
    private final RobotState robotState = RobotState.getInstance();
    private final Drive drive;
    private final AutoFactory factory;
    private final SubsystemScheduler<Superstructure> superstructure;
    private final SubsystemScheduler<Intake> intake;

    public AutoRoutines(
            Drive drive, SubsystemScheduler<Superstructure> superstructure, SubsystemScheduler<Intake> intake) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;

        factory = new AutoFactory(
                robotState::getPose, robotState::resetPose, drive::followPath, true, drive, (trajectory, starting) -> {
                    if (starting) {
                        var poses =
                                RobotState.isBlue() ? trajectory.getPoses() : GeometryUtil.flip(trajectory.getPoses());
                        Logger.recordOutput("Choreo/Trajectory", poses);
                    } else {
                        Logger.recordOutput("Choreo/Trajectory", new Pose2d[] {});
                        Logger.recordOutput("Choreo/TargetPose", Pose2d.kZero);
                    }
                });
    }

    private LoggedAutoRoutine create(String name) {
        return new LoggedAutoRoutine(name, factory, drive, superstructure, intake);
    }

    public AutoRoutine demo() {
        return create("Demo")
                .resetPose("Demo")
                .followPath("Demo")
                .driveToPose(new Pose2d(2.0, 2.0, Rotation2d.kZero))
                .build();
    }

    public AutoRoutine Left() {
        return create("Left")
                .requireStartingPose("Left")
                .intake()
                .setSuperstructureState(SuperstructureState.TRACKING_HUB_FOR_POSE)
                .followPath("Left")
                .shoot(11.4)
                .build();
    }

    public AutoRoutine Right() {
        return create("Right")
                .requireStartingPose("Right")
                .intake()
                .setSuperstructureState(SuperstructureState.TRACKING_HUB_FOR_POSE)
                .followPath("Right")
                .shoot(11.4)
                .build();
    }
}
