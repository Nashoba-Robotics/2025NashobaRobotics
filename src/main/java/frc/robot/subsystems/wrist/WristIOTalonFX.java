package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class WristIOTalonFX implements WristIO {
  private final TalonFX wrist;
  private final TalonFXConfiguration config;

  private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);

  public WristIOTalonFX() {
    wrist = new TalonFX(Constants.Wrist.WRIST_ID, Constants.Wrist.CANBUS);
    config = new TalonFXConfiguration();

    config.Slot0 = Constants.Wrist.PID;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Wrist.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Wrist.SUPPLY_LIMIT;

    config.MotorOutput.Inverted = Constants.Wrist.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Wrist.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Wrist.MOTION_MAGIC_ACCELERATION;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Wrist.FORWARD_SOFT_LIMIT.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Wrist.REVERSE_SOFT_LIMIT.getRotations();

    config.Feedback.FeedbackRemoteSensorID = Constants.Wrist.ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = Constants.Wrist.SENSOR_TO_MECHANISM_GEAR_RATIO;
    config.Feedback.RotorToSensorRatio = Constants.Wrist.ROTOR_TO_MECHANISM_GEAR_RATIO;

    wrist.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.connected = wrist.isConnected();
    inputs.positionRad = Units.rotationsToRadians(wrist.getPosition().getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(wrist.getVelocity().getValueAsDouble());
    inputs.appliedVolts = wrist.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = wrist.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = wrist.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = wrist.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setSetpoint(double setpointRads) {
    wrist.setControl(motionMagic.withPosition(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void setkV(double kV) {
    config.Slot0.kV = kV;
    wrist.getConfigurator().apply(config);
  }

  @Override
  public void setkP(double kP) {
    config.Slot0.kP = kP;
    wrist.getConfigurator().apply(config);
  }

  @Override
  public void setkD(double kD) {
    config.Slot0.kD = kD;
    wrist.getConfigurator().apply(config);
  }
}
