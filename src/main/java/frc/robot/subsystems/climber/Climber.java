package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public double setpoint;

  public Climber() {
    io = new ClimberIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public double getAngleRads() {
    return inputs.leaderPositionRad;
  }

  public Command deployClimber() {
    return run(() -> runSetpoint(-2))
        .until(() -> Util.epsilonEquals(inputs.leaderPositionRad, setpoint, 0.1));
  }

  public Command retractClimber() {
    return run(() -> runSetpoint(0.25))
        .until(() -> Util.epsilonEquals(inputs.leaderPositionRad, setpoint, 0.1));
  }

  public void runSetpoint(double setpointRads) {
    setpoint = setpointRads;
    io.runSetpoint(setpointRads);
  }

  public void runDutyCycle(double percent) {
    io.runDutyCycle(percent);
  }

  public void stop() {
    io.stop();
  }

  public void zeroClimber() {
    io.setPosition(0);
  }

  public void setPID(double kV, double kP, double kD) {
    io.setkV(kV);
    io.setkP(kP);
    io.setkD(kD);
  }
}
