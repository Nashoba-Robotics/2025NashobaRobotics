package frc.robot.subsystems.candle;
package com.ctre.phoenix

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.PhoenixUtil;
import com.

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;



public class candle extends SubsystemBase{
    private static CANdle candle;
    private RobotState lastState;

    private static final int LED_COUNT = 85;

    private boolean clearAnimationFlag = false;

    private Color red = new Color(255, 0, 0);
    private Color green = new Color(0, 255, 0);
    private Color blue = new Color(0, 0, 255);
    private Color disabled = new Color(0xFF, 0x10, 0x0);
    private Color intake = new Color(255, 0, 255);
    private Color reef = blue;
    private Color reefPrep = new Color(0, 255, 255);
    private Color processor = new Color(255, 105, 180);
    private Color shootPrep = new Color(255, 255, 0);
    private Color shoot = green;
    private Color misc = new Color(0x9, 0x22, 0x15);
    private Color climb = new Color(155, 155, 155);
}
