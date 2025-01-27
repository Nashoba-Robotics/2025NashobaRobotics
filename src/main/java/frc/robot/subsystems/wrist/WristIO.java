package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setSetpoint(double setpointRads) {}

  public default void setkV(double kV) {}

  public default void setkP(double kP) {}

  public default void setkD(double kD) {}
}
