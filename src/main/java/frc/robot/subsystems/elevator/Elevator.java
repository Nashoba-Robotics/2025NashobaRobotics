package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  public void setExtension(double setpointMeters) {
    io.setSetpoint(setpointMeters / Constants.Elevator.PULLY_RAIDUS);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getLeaderStatorCurrent() {
    return inputs.leaderStatorCurrentAmps;
  }

  public double getFollowerStatorCurrent() {
    return inputs.followerStatorCurrentAmps;
  }

  public void zeroElevator() {
    io.setPosition(0 / Constants.Elevator.PULLY_RAIDUS);
  }

  public void setPID(double kV, double kP, double kD) {
    io.setkV(kV);
    io.setkP(kP);
    io.setkD(kD);
  }

  public void stop() {
    io.stop();
  }
}
