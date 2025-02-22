package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class HopperIOTalonFX implements HopperIO {
  private final TalonFX hopper;
  private final TalonFXConfiguration config;

  public HopperIOTalonFX() {
    hopper = new TalonFX(Constants.Hopper.HOPPER_ID, Constants.Hopper.CANBUS);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Hopper.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hopper.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Manipulator.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Hopper.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    hopper.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.connected = hopper.isConnected();
    inputs.velocityRadPerSec = Units.rotationsToRadians(hopper.getVelocity().getValueAsDouble());
    inputs.appliedVolts = hopper.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = hopper.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = hopper.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = hopper.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void runPercentOutput(double percent) {
    hopper.setControl(new DutyCycleOut(percent));
  }
}
