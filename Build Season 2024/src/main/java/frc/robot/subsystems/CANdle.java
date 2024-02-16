// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.CANdleID, "DriveBus");

    public LEDState ledstate;

    public Candle() {

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        m_candle.configAllSettings(configAll, 100);
        m_candle.clearAnimation(0);
        ledstate = LEDState.WHITE;
        setLEDSTate(ledstate);
    }

    public void setLEDSTate(LEDState state) {

        ledstate = state;
        m_candle.setLEDs(state.r, state.g, state.b);
    }

    public LEDState getLEDState() {

        return ledstate;

    }

    public enum LEDState {

        ORANGE(Constants.ORANGE_R, Constants.ORANGE_G, Constants.ORANGE_B),
        GREEN(Constants.GREEN_R, Constants.GREEN_G, Constants.GREEN_B),
        PURPLE(Constants.PURPLE_R, Constants.PURPLE_G, Constants.PURPLE_B),
        YELLOW(Constants.YELLOW_R, Constants.YELLOW_G, Constants.YELLOW_B),
        RED(Constants.RED_R, Constants.RED_G, Constants.RED_B),
        BLACK(Constants.NONE_R, Constants.NONE_G, Constants.NONE_B),
        WHITE(Constants.WHITE_R, Constants.WHITE_G, Constants.WHITE_B),
        COPPER(Constants.COPPER_R, Constants.COPPER_G, Constants.COPPER_B)

        ;

        public final int r;
        public final int g;
        public final int b;

        private LEDState(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LED_R", getLEDState().r);
        SmartDashboard.putNumber("LED_G", getLEDState().g);
        SmartDashboard.putNumber("LED_B", getLEDState().b);

    }

}