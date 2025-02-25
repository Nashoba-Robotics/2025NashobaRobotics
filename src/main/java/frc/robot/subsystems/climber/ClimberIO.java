package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean leaderConnected = false;
    public double leaderPositionRad = 0.0;
    public double leaderVelocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerPositionRad = 0.0;
    public double followerVelocityRadPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runSetpoint(double setpointRads) {}

  public default void runDutyCycle(double percent) {}

  public default void stop() {}

  public default void setPosition(double angleRads) {}

  public default void setkV(double kV) {}

  public default void setkP(double kP) {}

  public default void setkD(double kD) {}
}
