package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public double setpoint;

  public Wrist() {
    io = new WristIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    Logger.recordOutput("Wristsetpoint", setpoint);
  }

  public void runSetpoint(double setpointRads) {
    this.setpoint = setpointRads;
    io.runSetpoint(setpointRads);
  }

  public void runDutyCycle(double percent) {
    io.runDutyCycle(percent);
  }

  public double getAngleRads() {
    return inputs.absolutePositionRad;
  }

  public double getSetpointRads() {
    return setpoint;
  }

  public Command runAngleCommand(double setpointRads) {
    return run(() -> runSetpoint(setpointRads))
        .until(
            () ->
                Util.epsilonEquals(
                    getPositionRadians(),
                    setpointRads,
                    Constants.Wrist.WRIST_TOLERANCE.getRadians()));
  }

  public Command runExtensionCommand(double setpointRads, double thresholdRads) {
    return run(() -> runSetpoint(setpointRads)).until(() -> getPositionRadians() >= thresholdRads);
  }

  public Command runTuckCommand(double setpointRads, double thresholdRads) {
    return run(() -> runSetpoint(setpointRads)).until(() -> getPositionRadians() <= thresholdRads);
  }

  public double getPositionRadians() {
    return inputs.rotorPositionRad;
  }

  public void stop() {
    io.stop();
  }

  public void setPID(double kV, double kP, double kD) {
    io.setkV(kV);
    io.setkP(kP);
    io.setkD(kD);
  }
}
