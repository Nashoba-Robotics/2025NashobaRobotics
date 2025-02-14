package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leader;
  private final TalonFX follower;
  private final TalonFXConfiguration config;

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
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Elevator.FORWARD_SOFT_LIMIT.getRotations();

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(leader.getPosition().getValueAsDouble());
    inputs.positionMeters =
        Units.rotationsToRadians(leader.getPosition().getValueAsDouble())
            * Constants.Elevator.PULLY_RAIDUS;
    inputs.velocityMetersPerSec =
        Units.rotationsToRadians(leader.getVelocity().getValueAsDouble())
            * Constants.Elevator.PULLY_RAIDUS;

    inputs.leaderMotorConnected = leader.isConnected();
    inputs.leaderAppliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
    inputs.leaderTempCelsius = leader.getDeviceTemp().getValueAsDouble();

    inputs.followerMotorConnected = follower.isConnected();
    inputs.followerAppliedVolts = follower.getMotorVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
    inputs.followerStatorCurrentAmps = follower.getStatorCurrent().getValueAsDouble();
    inputs.followerTempCelsius = follower.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setSetpoint(double setpointRads) {
    leader.setControl(new MotionMagicDutyCycle(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void setPosition(double rotations) {
    leader.setPosition(rotations);
  }

  @Override
  public void setDutyCycle(double percent) {
    leader.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(new VoltageOut(voltage));
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
