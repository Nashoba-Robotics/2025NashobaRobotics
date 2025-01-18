package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ManipulatorIOTalonFX implements ManipulatorIO {
  private final TalonFX manipulator;
  private final TalonFXConfiguration config;

  private final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);

  public ManipulatorIOTalonFX() {
    manipulator = new TalonFX(Constants.Manipulator.MANIPULATOR_ID, Constants.Manipulator.CANBUS);
    config = new TalonFXConfiguration();

    config.Slot0 = Constants.Manipulator.PID;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Manipulator.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Manipulator.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Manipulator.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Manipulator.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicAcceleration = Constants.Manipulator.MOTION_MAGIC_ACCELERATION;

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
  public void setSetpoint(double setpointRadsPerSec) {
    manipulator.setControl(
        velocityControl.withVelocity(Units.radiansToRotations(setpointRadsPerSec)));
  }

  @Override
  public void setkP(double kP) {
    config.Slot0.kP = kP;
    manipulator.getConfigurator().apply(config);
  }

  @Override
  public void setkD(double kD) {
    config.Slot0.kD = kD;
    manipulator.getConfigurator().apply(config);
  }

  @Override
  public void setkV(double kV) {
    config.Slot0.kV = kV;
    manipulator.getConfigurator().apply(config);
  }

  @Override
  public void setkS(double kS) {
    config.Slot0.kS = kS;
    manipulator.getConfigurator().apply(config);
  }
}
