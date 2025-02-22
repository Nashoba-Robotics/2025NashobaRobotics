package frc.robot.subsystems.manipulator;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ManipulatorIOTalonFX implements ManipulatorIO {
  private final TalonFX manipulator;
  private final TalonFXConfiguration config;

  private final LaserCan sensor;

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private NeutralOut neutralOut = new NeutralOut();

  public ManipulatorIOTalonFX() {
    manipulator = new TalonFX(Constants.Manipulator.MANIPULATOR_ID, Constants.Manipulator.CANBUS);
    config = new TalonFXConfiguration();

    sensor = new LaserCan(Constants.Manipulator.SENSOR_ID);
    try {
      sensor.setRangingMode(LaserCan.RangingMode.SHORT);
      sensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

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

    inputs.coralPresent =
        (sensor.getMeasurement() != null
            && sensor.getMeasurement().distance_mm
                <= Constants.Manipulator.SENSOR_DISTANCE_THRESHOLD);
  }

  @Override
  public void runPercentOutput(double percent) {
    manipulator.setControl(dutyCycleOut.withOutput(percent));
  }

  @Override
  public void stop() {
    manipulator.setControl(neutralOut);
  }
}
