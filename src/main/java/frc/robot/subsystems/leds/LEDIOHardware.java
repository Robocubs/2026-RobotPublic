// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import frc.robot.PhoenixCANBus;
import frc.robot.util.PhoenixUtil;

public class LEDIOHardware implements LEDIO {
    private final CANdle candle = new CANdle(0, PhoenixCANBus.canivore());

    public LEDIOHardware() {
        var config = new CANdleConfiguration()
                .withCANdleFeatures(new CANdleFeaturesConfigs().withVBatOutputMode(VBatOutputModeValue.Modulated))
                .withLED(new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(1.0));
        PhoenixUtil.tryUntilOk(() -> candle.getConfigurator().apply(config));
    }

    @Override
    public void animate(ControlRequest animation) {
        candle.setControl(animation);
    }
}
