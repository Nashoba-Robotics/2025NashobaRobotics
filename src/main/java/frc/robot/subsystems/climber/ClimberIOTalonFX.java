package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX pivot;
  private final TalonFX grabber;
  private final TalonFXConfiguration pivotConfig;
  private final TalonFXConfiguration grabberConfig;

  private final MotionMagicDutyCycle angleControl = new MotionMagicDutyCycle(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public ClimberIOTalonFX() {
    pivot = new TalonFX(Constants.Climber.PIVOT_ID, Constants.Climber.CANBUS);
    grabber = new TalonFX(Constants.Climber.GRABBER_ID, Constants.Climber.CANBUS);

    pivotConfig = new TalonFXConfiguration();
    grabberConfig = new TalonFXConfiguration();

    pivotConfig.Slot0 = Constants.Climber.PID;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Climber.PIVOT_STATOR_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climber.PIVOT_SUPPLY_LIMIT;

    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.Climber.PIVOT_GEAR_RATIO;

    pivotConfig.MotorOutput.Inverted = Constants.Climber.PIVOT_INVERTED;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Climber.PIVOT_MOTION_MAGIC_CRUISE_VELOCITY;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        Constants.Climber.PIVOT_MOTION_MAGIC_ACCELERATION;

    grabberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    grabberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    grabberConfig.CurrentLimits.StatorCurrentLimit = Constants.Climber.PIVOT_STATOR_LIMIT;
    grabberConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climber.PIVOT_SUPPLY_LIMIT;

    grabberConfig.MotorOutput.Inverted = Constants.Climber.GRABBER_INVERTED;
    grabberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    grabberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    grabberConfig.Feedback.SensorToMechanismRatio = Constants.Climber.GRABBER_GEAR_RATIO;

    pivot.getConfigurator().apply(pivotConfig);
    grabber.getConfigurator().apply(grabberConfig);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.pivotConnected = pivot.isConnected();
    inputs.pivotPositionRad = Units.rotationsToRadians(pivot.getPosition().getValueAsDouble());
    inputs.pivotVelocityRadPerSec =
        Units.rotationsToRadians(pivot.getPosition().getValueAsDouble());
    inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivot.getSupplyCurrent().getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivot.getStatorCurrent().getValueAsDouble();
    inputs.pivotTempCelsius = pivot.getDeviceTemp().getValueAsDouble();

    inputs.grabberConnected = grabber.isConnected();
    inputs.grabberPositionRad = Units.rotationsToRadians(pivot.getPosition().getValueAsDouble());
    inputs.grabberVelocityRadPerSec =
        Units.rotationsToRadians(pivot.getPosition().getValueAsDouble());
    inputs.grabberAppliedVolts = grabber.getMotorVoltage().getValueAsDouble();
    inputs.grabberSupplyCurrentAmps = grabber.getSupplyCurrent().getValueAsDouble();
    inputs.grabberStatorCurrentAmps = grabber.getStatorCurrent().getValueAsDouble();
    inputs.grabberTempCelsius = grabber.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPivotSetpoint(double setpointRads) {
    pivot.setControl(angleControl.withPosition(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void setGrabberVoltage(double setpointVoltage) {
    grabber.setControl(voltageControl.withOutput(setpointVoltage));
  }

  @Override
  public void setPivotPosition(double rads) {
    pivot.setPosition(Units.radiansToRotations(rads));
  }

  @Override
  public void setkV(double kV) {
    pivotConfig.Slot0.kV = kV;
    pivot.getConfigurator().apply(pivotConfig);
  }
  
  @Override
  public void setkP(double kP) {
    pivotConfig.Slot0.kP = kP;
    pivot.getConfigurator().apply(pivotConfig);
  }

  @Override
  public void setkD(double kD) {
    pivotConfig.Slot0.kD = kD;
    pivot.getConfigurator().apply(pivotConfig);
  }
}
