package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leader;
  private final TalonFX follower;
  private final TalonFXConfiguration config;

  private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);

  public ElevatorIOTalonFX() {
    leader = new TalonFX(Constants.Elevator.LEADER_ID, Constants.Elevator.CANBUS);
    follower = new TalonFX(Constants.Elevator.FOLLOWER_ID, Constants.Elevator.CANBUS);
    follower.setControl(new Follower(Constants.Elevator.LEADER_ID, false));

    config = new TalonFXConfiguration();

    config.Slot0 = Constants.Elevator.PID;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Elevator.STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Elevator.GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Elevator.INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Elevator.MOTION_MAGIC_ACCELERATION;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Elevator.FORWARD_SOFT_LIMIT.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Elevator.REVERSE_SOFT_LIMIT.getRotations();

    leader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionMeters = leader.getPosition().getValueAsDouble();
    inputs.velocityMetersPerSec = leader.getVelocity().getValueAsDouble();

    inputs.leaderMotorConnected = leader.isConnected();
    inputs.leaderAppliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
    inputs.leaderTempCelsius = leader.getDeviceTemp().getValueAsDouble();

    inputs.followerMotorConnected = follower.isConnected();
    inputs.followerAppliedVolts = follower.getMotorVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.followerStatorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
    inputs.followerTempCelsius = leader.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setSetpoint(double setpointMeters) {
    leader.setControl(motionMagic.withPosition(setpointMeters));
  }

  @Override
  public void setPosition(double meters) {
    leader.setPosition(meters);
  }
  
  @Override
  public void setkV(double kV) {
    config.Slot0.kV = kV;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void setkP(double kP) {
    config.Slot0.kP = kP;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void setkD(double kD) {
    config.Slot0.kD = kD;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
      leader.setControl(new NeutralOut());
  }
}
