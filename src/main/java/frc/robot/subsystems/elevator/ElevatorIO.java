package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;

    public boolean leaderMotorConnected = false;
    public double leaderAppliedVolts = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public boolean followerMotorConnected = false;
    public double followerAppliedVolts = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setSetpoint(double setpointMeters) {}

  public default void setPosition(double meters) {}

  public default void setkS(double kS) {}

  public default void setkG(double kG) {}

  public default void setkV(double kV) {}

  public default void setkP(double kP) {}

  public default void setkD(double kD) {}
}
