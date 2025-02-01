package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public double setpointMeters = 0;

  public Elevator() {
    io = new ElevatorIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    
    Logger.recordOutput("Elevator setpoint", setpointMeters);
  }

  public void setExtension(double setpointMeters) {
    this.setpointMeters = setpointMeters;
    io.setSetpoint(setpointMeters / Constants.Elevator.PULLY_RAIDUS);
  }

  public double getPositionMeters(){
    return inputs.positionMeters;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void zeroElevator() {
    io.setPosition(0 / Constants.Elevator.PULLY_RAIDUS);
  }

  public Command setExtensionCommand(double setpointMeters) {
    return run(() -> setExtension(setpointMeters)).until(() -> Util.epsilonEquals(getPositionMeters(), setpointMeters, Constants.Elevator.ELEVATOR_TOLERANCE));
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
