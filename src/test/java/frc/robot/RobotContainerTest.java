// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

class RobotContainerTest {

    @BeforeAll
    static void initAll() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void canBeConstructed() {
        assertDoesNotThrow(RobotContainer::new);
    }
}
