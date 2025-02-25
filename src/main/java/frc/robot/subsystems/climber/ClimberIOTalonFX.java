package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final TalonFXConfiguration config;

  private final MotionMagicDutyCycle angleControl = new MotionMagicDutyCycle(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public ClimberIOTalonFX() {
    leader = new TalonFX(Constants.Climber.LEADER_ID, Constants.Climber.CANBUS);
    follower = new TalonFX(Constants.Climber.FOLLOWER_ID, Constants.Climber.CANBUS);
    follower.setControl(new Follower(Constants.Climber.LEADER_ID, true));

    config = new TalonFXConfiguration();

    config.Slot0 = Constants.Climber.PID;

    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = Constants.Climber.PIVOT_STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Climber.PIVOT_SUPPLY_LIMIT;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Climber.PIVOT_GEAR_RATIO;

    config.MotorOutput.Inverted = Constants.Climber.PIVOT_INVERTED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Climber.PIVOT_MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climber.PIVOT_MOTION_MAGIC_ACCELERATION;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leaderConnected = leader.isConnected();
    inputs.leaderPositionRad = Units.rotationsToRadians(leader.getPosition().getValueAsDouble());
    inputs.leaderVelocityRadPerSec =
        Units.rotationsToRadians(leader.getPosition().getValueAsDouble());
    inputs.leaderAppliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
    inputs.leaderTempCelsius = leader.getDeviceTemp().getValueAsDouble();

    inputs.followerConnected = follower.isConnected();
    inputs.followerPositionRad =
        Units.rotationsToRadians(follower.getPosition().getValueAsDouble());
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(follower.getPosition().getValueAsDouble());
    inputs.followerAppliedVolts = follower.getMotorVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
    inputs.followerStatorCurrentAmps = follower.getStatorCurrent().getValueAsDouble();
    inputs.followerTempCelsius = follower.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void runSetpoint(double setpointRads) {
    leader.setControl(angleControl.withPosition(Units.radiansToRotations(setpointRads)));
  }

  @Override
  public void runDutyCycle(double percent) {
    leader.setControl(dutyCycleOut.withOutput(percent));
  }

  @Override
  public void stop() {
    leader.setControl(new NeutralOut());
  }

  @Override
  public void setPosition(double angleRads) {
    leader.setPosition(Units.radiansToRotations(angleRads));
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
}
