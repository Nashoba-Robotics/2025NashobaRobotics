package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ManipulatorIOTalonFX implements ManipulatorIO {
  private final TalonFX manipulator;
  private final TalonFXConfiguration config;

  public ManipulatorIOTalonFX() {
    manipulator = new TalonFX(Constants.Manipulator.MANIPULATOR_ID, Constants.Manipulator.CANBUS);
    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Manipulator.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Manipulator.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Manipulator.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Manipulator.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    manipulator.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.connected = manipulator.isConnected();
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(manipulator.getVelocity().getValueAsDouble());
    inputs.appliedVolts = manipulator.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = manipulator.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = manipulator.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = manipulator.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVoltage(double setpointVolts) {
    manipulator.setControl(new VoltageOut(setpointVolts));
  }
}
