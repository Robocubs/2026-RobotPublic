// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MatchState;
import frc.robot.RobotState;

import static edu.wpi.first.units.Units.Seconds;

public class LED extends SubsystemBase {
    private static final int ledStartIdx = 0;
    private static final int ledEndIdx = 600;

    private final LEDIO io;
    private final RobotState robotState = RobotState.getInstance();
    private ControlRequest toAnimate = null;
    private Color color = Color.kWhite;
    private AnimationType animationType = null;

    public enum AnimationType {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }

    public LED(LEDIO io) {
        this.io = io;
        changeAnimation(AnimationType.ColorFlow, Color.kWhite);
    }

    @Override
    public void periodic() {
        if (robotState.getMatchState().getPhase() == MatchState.Phase.DISABLED) {
            changeAnimation(AnimationType.SetAll, Color.kMaroon);
        } else if (robotState.getMatchState().getPhase() == MatchState.Phase.AUTO) {
            changeAnimation(AnimationType.SetAll, Color.kWhite);
        } else if (robotState.getMatchState().getTimeToHubActive().lt(Seconds.of(7))) {
            changeAnimation(AnimationType.Strobe, Color.kRed);
        } else if (robotState.getMatchState().getTimeToHubInactive().lt(Seconds.of(5))) {
            changeAnimation(AnimationType.Strobe, Color.kBlue);
        } else if (robotState.getMatchState().isHubActive()) {
            changeAnimation(AnimationType.SetAll, Color.kBlue);
        } else if (robotState.getMatchState().isHubActive() == false) {
            changeAnimation(AnimationType.SetAll, Color.kRed);
        }

        if (toAnimate != null) {
            io.animate(toAnimate);
        } else {
            io.animate(new SolidColor(ledStartIdx, ledEndIdx).withColor(new RGBWColor(Color.kWhite)));
        }
    }

    private void changeAnimation(AnimationType animation, Color color) {
        if (animationType == animation && this.color == color) {
            return;
        }

        animationType = animation;
        this.color = color;

        // Convert WPILib Color to RGBWColor for Phoenix 6
        RGBWColor rgbwColor = new RGBWColor(color);

        switch (animation) {
            case ColorFlow:
                toAnimate = new ColorFlowAnimation(ledStartIdx, ledEndIdx)
                        .withColor(rgbwColor)
                        .withFrameRate(0.7);
                break;
            case Fire:
                toAnimate = new FireAnimation(ledStartIdx, ledEndIdx)
                        .withBrightness(0.5)
                        .withSparking(0.7)
                        .withCooling(0.5);
                break;
            case Larson:
                toAnimate = new LarsonAnimation(ledStartIdx, ledEndIdx)
                        .withColor(rgbwColor)
                        .withSize(3)
                        .withBounceMode(LarsonBounceValue.Front)
                        .withFrameRate(0.1);
                break;
            case Rainbow:
                toAnimate = new RainbowAnimation(ledStartIdx, ledEndIdx)
                        .withBrightness(1.0)
                        .withFrameRate(0.1);
                break;
            case RgbFade:
                toAnimate = new RgbFadeAnimation(ledStartIdx, ledEndIdx)
                        .withBrightness(0.7)
                        .withFrameRate(0.4);
                break;
            case SingleFade:
                toAnimate = new SingleFadeAnimation(ledStartIdx, ledEndIdx)
                        .withColor(rgbwColor)
                        .withFrameRate(0.5);
                break;
            case Strobe:
                toAnimate = new StrobeAnimation(ledStartIdx, ledEndIdx)
                        .withColor(rgbwColor)
                        .withFrameRate(0.98);
                break;
            case Twinkle:
                toAnimate = new TwinkleAnimation(ledStartIdx, ledEndIdx)
                        .withColor(rgbwColor)
                        .withMaxLEDsOnProportion(0.06)
                        .withFrameRate(0.4);
                break;
            case TwinkleOff:
                toAnimate = new TwinkleOffAnimation(ledStartIdx, ledEndIdx)
                        .withColor(rgbwColor)
                        .withMaxLEDsOnProportion(1.0)
                        .withFrameRate(0.8);
                break;
            case SetAll:
                toAnimate = new SolidColor(ledStartIdx, ledEndIdx).withColor(rgbwColor);
                break;
        }
    }
}
