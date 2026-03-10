// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

/**
 * A trapezoid profile controller based off of WPILib's ProfiledPIDController, but without the PID
 */
public class TrapezoidProfileController {
    private TrapezoidProfile profile;
    private boolean continuous;
    private double minimumInput;
    private double maximumInput;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0.0, 0.0);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0.0, 0.0);

    public TrapezoidProfileController(TrapezoidProfile.Constraints constraints) {
        this.profile = new TrapezoidProfile(constraints);
    }

    public TrapezoidProfileController(double maxVelocity, double maxAcceleration) {
        this(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

    public TrapezoidProfileController withContinuousInput(double minimumInput, double maximumInput) {
        this.continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
        return this;
    }

    public TrapezoidProfileController withConstraints(TrapezoidProfile.Constraints constraints) {
        this.profile = new TrapezoidProfile(constraints);
        return this;
    }

    public TrapezoidProfileController withConstraints(double maxVelocity, double maxAcceleration) {
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        return this;
    }

    public TrapezoidProfileController withGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
        return this;
    }

    public TrapezoidProfileController withGoal(double position, double velocity) {
        this.goal = new TrapezoidProfile.State(position, velocity);
        return this;
    }

    public TrapezoidProfileController withGoal(double position) {
        this.goal = new TrapezoidProfile.State(position, 0.0);
        return this;
    }

    public TrapezoidProfile.State calculate(double measurement) {
        if (continuous) {
            // Get error which is the smallest distance between goal and measurement
            var errorBound = (maximumInput - minimumInput) / 2.0;
            var goalMinDistance = MathUtil.inputModulus(goal.position - measurement, -errorBound, errorBound);
            var setpointMinDistance = MathUtil.inputModulus(setpoint.position - measurement, -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            goal.position = goalMinDistance + measurement;
            setpoint.position = setpointMinDistance + measurement;
        }

        setpoint = profile.calculate(Constants.mainLoopPeriod.in(Seconds), setpoint, goal);
        return setpoint;
    }

    public TrapezoidProfile.State calculate(double measurement, TrapezoidProfile.State goal) {
        this.goal = goal;
        return calculate(measurement);
    }

    public TrapezoidProfile.State calculate(double measurement, double goal) {
        this.goal = new TrapezoidProfile.State(goal, 0.0);
        return calculate(measurement);
    }

    public void reset(TrapezoidProfile.State measurement) {
        this.setpoint = measurement;
    }

    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }
}
