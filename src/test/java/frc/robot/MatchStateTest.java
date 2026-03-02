// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.epsilon;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MatchStateTest {

    private MatchState matchState;

    @BeforeAll
    static void initAll() {
        assert HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        matchState = new MatchState();
    }

    private void setTeleop(double matchTime) {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setMatchTime(matchTime);
        DriverStationSim.notifyNewData();
    }

    private void setWinner(boolean winnerIsRed, boolean weAreRed) {
        DriverStationSim.setGameSpecificMessage(winnerIsRed ? "R" : "B");
        DriverStationSim.setAllianceStationId(weAreRed ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    @Nested
    class PhaseTests {

        @Test
        void disabled_returnsDisabled() {
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            matchState.update();
            assertEquals(MatchState.Phase.DISABLED, matchState.getPhase());
        }

        @Test
        void autonomous_returnsAuto() {
            DriverStationSim.setEnabled(true);
            DriverStationSim.setAutonomous(true);
            DriverStationSim.notifyNewData();
            matchState.update();
            assertEquals(MatchState.Phase.AUTO, matchState.getPhase());
        }

        @Test
        void teleop_aboveTransitionEnd_returnsTransition() {
            setTeleop(145.0);
            matchState.update();
            assertEquals(MatchState.Phase.TRANSITION, matchState.getPhase());
        }

        @Test
        void teleop_atTransitionEnd_returnsShift1() {
            setTeleop(130.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT1, matchState.getPhase());
        }

        @Test
        void teleop_midShift1_returnsShift1() {
            setTeleop(115.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT1, matchState.getPhase());
        }

        @Test
        void teleop_atShift1End_returnsShift2() {
            setTeleop(105.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT2, matchState.getPhase());
        }

        @Test
        void teleop_midShift2_returnsShift2() {
            setTeleop(90.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT2, matchState.getPhase());
        }

        @Test
        void teleop_atShift2End_returnsShift3() {
            setTeleop(80.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT3, matchState.getPhase());
        }

        @Test
        void teleop_midShift3_returnsShift3() {
            setTeleop(65.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT3, matchState.getPhase());
        }

        @Test
        void teleop_atShift3End_returnsShift4() {
            setTeleop(55.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT4, matchState.getPhase());
        }

        @Test
        void teleop_midShift4_returnsShift4() {
            setTeleop(40.0);
            matchState.update();
            assertEquals(MatchState.Phase.SHIFT4, matchState.getPhase());
        }

        @Test
        void teleop_atShift4End_returnsEndgame() {
            setTeleop(30.0);
            matchState.update();
            assertEquals(MatchState.Phase.ENDGAME, matchState.getPhase());
        }

        @Test
        void teleop_midEndgame_returnsEndgame() {
            setTeleop(10.0);
            matchState.update();
            assertEquals(MatchState.Phase.ENDGAME, matchState.getPhase());
        }

        @Test
        void teleop_negativeMatchTime_returnsEndgame() {
            setTeleop(-1.0);
            matchState.update();
            assertEquals(MatchState.Phase.ENDGAME, matchState.getPhase());
        }
    }

    @Nested
    class HubActiveTests {

        @Test
        void noWinner_shift1_hubActive() {
            setTeleop(115.0);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void noWinner_shift2_hubActive() {
            setTeleop(90.0);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void wonAuto_transition_hubActive() {
            setTeleop(140.0);
            setWinner(true, true);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void wonAuto_shift1_hubInactive() {
            setTeleop(115.0);
            setWinner(true, true);
            matchState.update();
            assertFalse(matchState.isHubActive());
        }

        @Test
        void wonAuto_shift2_hubActive() {
            setTeleop(90.0);
            setWinner(true, true);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void wonAuto_shift3_hubInactive() {
            setTeleop(65.0);
            setWinner(true, true);
            matchState.update();
            assertFalse(matchState.isHubActive());
        }

        @Test
        void wonAuto_shift4_hubActive() {
            setTeleop(40.0);
            setWinner(true, true);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void wonAuto_endgame_hubActive() {
            setTeleop(15.0);
            setWinner(true, true);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void lostAuto_shift1_hubActive() {
            setTeleop(115.0);
            setWinner(true, false);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void lostAuto_shift2_hubInactive() {
            setTeleop(90.0);
            setWinner(true, false);
            matchState.update();
            assertFalse(matchState.isHubActive());
        }

        @Test
        void lostAuto_shift3_hubActive() {
            setTeleop(65.0);
            setWinner(true, false);
            matchState.update();
            assertTrue(matchState.isHubActive());
        }

        @Test
        void lostAuto_shift4_hubInactive() {
            setTeleop(40.0);
            setWinner(true, false);
            matchState.update();
            assertFalse(matchState.isHubActive());
        }
    }

    @Nested
    class TimeToHubActiveTests {

        @Test
        void hubActive_returnsZero() {
            setTeleop(140.0);
            setWinner(true, true);
            matchState.update();
            assertEquals(0.0, matchState.getTimeToHubActive().in(Seconds));
        }

        @Test
        void noWinner_returnsZero() {
            setTeleop(115.0);
            matchState.update();
            assertEquals(0.0, matchState.getTimeToHubActive().in(Seconds));
        }

        @Test
        void wonAuto_shift1_returnsTimeRemainingInShift1() {
            setTeleop(120.0);
            setWinner(true, true);
            matchState.update();
            assertEquals(15.0, matchState.getTimeToHubActive().in(Seconds), epsilon);
        }

        @Test
        void wonAuto_shift3_returnsTimeRemainingInShift3() {
            setTeleop(65.0);
            setWinner(true, true);
            matchState.update();
            assertEquals(10.0, matchState.getTimeToHubActive().in(Seconds), epsilon);
        }

        @Test
        void lostAuto_shift2_returnsTimeRemainingInShift2() {
            setTeleop(90.0);
            setWinner(true, false);
            matchState.update();
            assertEquals(10.0, matchState.getTimeToHubActive().in(Seconds), epsilon);
        }

        @Test
        void lostAuto_shift4_returnsTimeRemainingInShift4() {
            setTeleop(40.0);
            setWinner(true, false);
            matchState.update();
            assertEquals(10.0, matchState.getTimeToHubActive().in(Seconds), epsilon);
        }
    }

    @Nested
    class TimeToHubInactiveTests {

        @Test
        void hubInactive_returnsZero() {
            setTeleop(115.0);
            setWinner(true, true);
            matchState.update();
            assertEquals(0.0, matchState.getTimeToHubInactive().in(Seconds));
        }

        @Test
        void noWinner_returnsRemainingMatchTime() {
            setTeleop(140.0);
            matchState.update();
            assertEquals(140.0, matchState.getTimeToHubInactive().in(Seconds), epsilon);
        }

        @Test
        void wonAuto_transition_returnsTimeRemainingInTransition() {
            setTeleop(140.0);
            setWinner(true, true);
            matchState.update();
            assertEquals(10.0, matchState.getTimeToHubInactive().in(Seconds), epsilon);
        }

        @Test
        void wonAuto_shift2_returnsTimeRemainingInShift2() {
            setTeleop(90.0);
            setWinner(true, true);
            matchState.update();
            assertEquals(10.0, matchState.getTimeToHubInactive().in(Seconds), epsilon);
        }

        @Test
        void lostAuto_shift1_returnsTimeRemainingInShift1() {
            setTeleop(120.0);
            setWinner(true, false);
            matchState.update();
            assertEquals(15.0, matchState.getTimeToHubInactive().in(Seconds), epsilon);
        }

        @Test
        void lostAuto_shift3_returnsTimeRemainingInShift3() {
            setTeleop(65.0);
            setWinner(true, false);
            matchState.update();
            assertEquals(10.0, matchState.getTimeToHubInactive().in(Seconds), epsilon);
        }
    }
}
