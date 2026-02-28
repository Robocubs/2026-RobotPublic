// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

public class PhoenixCANBus {
    public static CANBus rio() {
        return CANBus.roboRIO();
    }

    public static CANBus canivore() {
        return new CANBus("Default");
    }
}
