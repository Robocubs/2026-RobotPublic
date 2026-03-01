// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.util.UnitsUtil;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
    public static final LoggedTunableNumber rotationP = new LoggedTunableNumber("Drive/RotationP", 7.0);
    public static final LoggedTunableNumber rotationD = new LoggedTunableNumber("Drive/RotationD", 0.04);
    public static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> tunableSmoothSpeed =
            new LoggedTunableMeasure<>("Drive/SmoothSpeed", MetersPerSecond.of(1.5));
    public static final LoggedTunableMeasure<LinearAccelerationUnit, LinearAcceleration> tunableSmoothAcceleration =
            new LoggedTunableMeasure<>(
                    "Drive/SmoothAcceleration", tunableSmoothSpeed.get().div(Seconds.of(0.25)));

    public static final Distance driveBaseRadius = Meters.of(Math.hypot(
                    TunerConstants.FrontLeft.LocationX - TunerConstants.BackRight.LocationX,
                    TunerConstants.FrontLeft.LocationY - TunerConstants.BackRight.LocationY)
            / 2.0);
    public static final Distance wheelRadius = Meters.of(TunerConstants.FrontLeft.WheelRadius);

    public static final SwerveDrivetrainConstants drivetrainConstants = TunerConstants.DrivetrainConstants;
    public static final SwerveModuleConstants<?, ?, ?>[] moduleConstants = new SwerveModuleConstants[] {
        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight,
    };

    public static final Translation2d[] moduleTranslations = java.util.Arrays.stream(moduleConstants)
            .map(module -> new Translation2d(module.LocationX, module.LocationY))
            .toArray(Translation2d[]::new);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    public static final Matrix<N3, N1> odometryStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
    public static final Matrix<N3, N1> visionStdDevs = new Matrix<>(VecBuilder.fill(1, 1, 1));

    public static final LinearVelocity maxSpeed = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration maxAcceleration =
            maxSpeed.div(Seconds.of(0.33)); // Future: calculate based on robot mass/kT/slip current

    public static final AngularVelocity maxAngularVelocity = UnitsUtil.toAngularVelocity(maxSpeed, driveBaseRadius);
    public static final AngularAcceleration maxAngularAcceleration = maxAngularVelocity.div(Seconds.of(0.33));

    public static final double translationDeadband = 0.1;
    public static final double rotationDeadband = 0.1;
}
