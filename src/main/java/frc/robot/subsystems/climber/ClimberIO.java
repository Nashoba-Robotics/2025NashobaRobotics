package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean pivotConnected = false;
    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runSetpoint(double setpointRads) {}

  public default void setPosition(double angleRads) {}

  public default void setkV(double kV) {}

  public default void setkP(double kP) {}

  public default void setkD(double kD) {}
}
