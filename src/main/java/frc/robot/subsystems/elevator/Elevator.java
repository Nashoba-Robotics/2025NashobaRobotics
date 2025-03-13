package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public void runExtension(double setpointMeters) {
    this.setpointMeters = setpointMeters;
    io.runPosition(setpointMeters / Constants.Elevator.PULLY_RAIDUS);
  }

  public void runDutyCycle(double percent) {
    io.runDutyCycle(percent);
  }

  public void runVoltage(double voltage) {
    io.runVoltage(voltage);
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public double getSetpointMeters() {
    return setpointMeters;
  }

  public void zeroElevator() {
    io.setPosition(0 / Constants.Elevator.PULLY_RAIDUS);
  }

  public boolean isNearL4() {
    return getPositionMeters() > 0.90;
  }

  public boolean isNearL3() {
    return getPositionMeters() > 0.50;
  }

  public Command runSetpointCommand(double setpointMeters) {
    return run(() -> runExtension(setpointMeters))
        .until(
            () ->
                Util.epsilonEquals(
                    getPositionMeters(), setpointMeters, Constants.Elevator.ELEVATOR_TOLERANCE));
  }

  public Command runExtensionCommand(double setpointMeters, double thresholdMeters) {
    return run(() -> runExtension(setpointMeters))
        .until(() -> getPositionMeters() >= thresholdMeters);
  }

  public Command runTuckCommand(double setpointMeters, double thresholdMeters) {
    return run(() -> runExtension(setpointMeters))
        .until(() -> getPositionMeters() <= thresholdMeters);
  }

  public Command runNeutralPrep() {
    return new InstantCommand(
        () -> {
          if (getPositionMeters() >= 0.275) {
            runExtension(0.275);
          }
        },
        this);
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
