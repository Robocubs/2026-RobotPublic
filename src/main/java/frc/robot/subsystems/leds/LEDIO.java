// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;

public interface LEDIO {
    public default void animate(ControlRequest animation) {}
}
