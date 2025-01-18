package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator() {
    io = new ElevatorIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setSetpoint(double setpointMeters) {
    io.setSetpoint(setpointMeters);
  }

  public double getLeaderStatorCurrent() {
    return inputs.leaderStatorCurrentAmps;
  }

  public double getFollowerStatorCurrent() {
    return inputs.followerStatorCurrentAmps;
  }

  public void zeroElevator() {
    io.setPosition(0);
  }

  public void setPID(double kP, double kD, double kS, double kG, double kV) {
    io.setkP(kP);
    io.setkD(kD);
    io.setkS(kS);
    io.setkG(kG);
    io.setkV(kV);
  }
}
