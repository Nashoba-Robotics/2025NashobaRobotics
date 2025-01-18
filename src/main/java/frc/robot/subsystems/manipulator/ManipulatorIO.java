package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {

  @AutoLog
  public static class ManipulatorIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setSetpoint(double setpointRadsPerSec) {}

  public default void setkP(double kP) {}

  public default void setkD(double kD) {}

  public default void setkV(double kV) {}

  public default void setkS(double kS) {}
}
