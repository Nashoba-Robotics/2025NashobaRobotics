package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureGoal;

public class LEDSubsystem extends SubsystemBase {
  private CANdle candle;
  private Superstructure superstructure;

  private final int LED_COUNT = 40;
  private boolean clearAnimationFlag = false;

  private RainbowAnimation rainbow = new RainbowAnimation(1, 0.8, LED_COUNT);
  private LarsonAnimation disabled =
      new LarsonAnimation(230, 30, 0, 0, 0.01, LED_COUNT, BounceMode.Back, 7);
  private FireAnimation fire =
      // new FireAnimation(1, 0.75, 16, 1, 0.5, false, 8);
      new FireAnimation(1, 0.75, LED_COUNT, 1, 0.5, false, 20);

  public LEDSubsystem(Superstructure superstructure) {
    candle = new CANdle(0, "rio");
    this.superstructure = superstructure;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) setAnimation(rainbow);
    else {
      if (DriverStation.isAutonomous()) setAnimation(rainbow);
      else {
        if (clearAnimationFlag) {
          candle.clearAnimation(0);
          clearAnimationFlag = false;
        }

        if (superstructure.getGoal() == SuperstructureGoal.NEUTRAL) candle.setLEDs(255, 255, 255);
        else if (superstructure.getGoal() == SuperstructureGoal.INTAKE) {
          if (superstructure.hasCoral()) candle.setLEDs(0, 255, 0);
          else candle.setLEDs(153, 51, 255);
        }
      }
    }
  }

  public void setAnimation(Animation animation) {
    clearAnimationFlag = true;
    candle.animate(animation);
  }
}
