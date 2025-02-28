package frc.robot.subsystems.candle;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;

/*
Notes & ToDo:
16 LEDs on strips
Need dedicated auto rainbow animation
Set perameters for animations
Figure out what a larson animation is

Impliment means for color use`
Discover Animation "numLED" perameter indexing: From zero?Refine colors & cut clutter
*/

public class Candle extends SubsystemBase {

  private CANdle candleLEDs;
  private Superstructure superStructure;
  private boolean lastDisabledState;
  private boolean lastAutoState;

  public Candle(Superstructure superStructure) {
    candleLEDs = new CANdle(0);
    this.superStructure = superStructure;
    lastDisabledState = true;
    lastAutoState = false;
    candleLEDs.animate(larsonFront);
  }

  // 2025 isn't a stated robot, prob to delete
  private RobotState lastState;

  private static final int LED_COUNT = 32;

  // This is ripped from 2024, no idea what it does but it might be important so it stays
  private boolean clearAnimationFlag = false;

  // Some of these use hexadecimal, why?
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

  // these are prob right
  // Other animations:
  // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/package-summary.html
  private Animation rainbow = new RainbowAnimation(1, 0.75, LED_COUNT, false, 0);
  private Animation fire = new FireAnimation(1, 0.75, LED_COUNT, 0.25, 0.125, false, 0);
  private Animation larsonBack =
      new LarsonAnimation(155, 155, 155, 1, 0.75, LED_COUNT, LarsonAnimation.BounceMode.Back, 3, 0);
  private Animation larsonCenter =
      new LarsonAnimation(155, 155, 155, 1, 0.75, LED_COUNT, LarsonAnimation.BounceMode.Center, 3, 0);
  private Animation larsonFront =
      new LarsonAnimation(155, 155, 155, 1, 0.75, LED_COUNT, LarsonAnimation.BounceMode.Front, 3, 0);

  @Override
  public void periodic() {
    System.out.println("Start LED Periodic");
    if (DriverStation.isDisabled() != lastDisabledState) {
      candleLEDs.animate(fire);
      lastDisabledState = DriverStation.isDisabled();
      System.out.println("End LED State");
    } else if (DriverStation.isDisabled() == false
        && DriverStation.isAutonomous() != lastAutoState) {
      candleLEDs.animate(rainbow);
      lastAutoState = DriverStation.isAutonomous();
      System.out.println("Auto LED State");
    }
    // manipulator was made public, problem?
    else if (superStructure.manipulator.isCoralPresent() == true
        && DriverStation.isAutonomous() == false) {
      candleLEDs.setLEDs(0, 0, 255, 175, 0, LED_COUNT);
      System.out.println("Coral LED State");
    } else if (DriverStation.isAutonomous() == false) {
      candleLEDs.setLEDs(0, 0, 0, 0, 0, LED_COUNT);
      System.out.println("Defualt LED State");
    } else {
      System.out.println("Initial LED State (Probably)");
    }
    System.out.println("End LED Periodic");
  }

  public void setCANdle(int state) {

    if (state == 0) {
      candleLEDs.animate(rainbow);
    } else if (state == 1) {
      candleLEDs.animate(fire);
    } else if (state == 2) {
      candleLEDs.animate(larsonBack);
    } else if (state == 3) {
      candleLEDs.animate(larsonCenter);
    } else if (state == 4) {
      candleLEDs.animate(larsonFront);
    }
  }

  public void setRainbow() {
    candleLEDs.animate(rainbow);
  }

  public void setFire() {
    candleLEDs.animate(fire);
  }

  public void setLarsonBack() {
    candleLEDs.animate(larsonBack);
  }

  public void setLarsonCenter() {
    candleLEDs.animate(larsonCenter);
  }

  public void setLarsonFront() {
    candleLEDs.animate(larsonFront);
  }

  public void off() {
    candleLEDs.setLEDs(0, 0, 0, 0, 0, LED_COUNT);
  }
}
