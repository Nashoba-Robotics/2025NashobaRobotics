package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX pivot;
  private final TalonFXConfiguration pivotConfig;

  private final MotionMagicDutyCycle angleControl = new MotionMagicDutyCycle(0);

  public ClimberIOTalonFX() {
    pivot = new TalonFX(Constants.Climber.PIVOT_ID, Constants.Climber.CANBUS);

    pivotConfig = new TalonFXConfiguration();

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

    pivot.getConfigurator().apply(pivotConfig);
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
  }

  @Override
  public void runSetpoint(double setpointRads) {
    pivot.setControl(angleControl.withPosition(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void setPosition(double angleRads) {
    pivot.setPosition(Units.radiansToRotations(angleRads));
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
