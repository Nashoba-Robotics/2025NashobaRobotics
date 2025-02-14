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
  }

  public void setSetpoint(double setpointRads) {
    this.setpoint = setpointRads;
    io.setSetpoint(setpointRads);
  }

  public double getAngleRads() {
    return inputs.absolutePositionRad;
  }

  public double getSetpointRads() {
    return setpoint;
  }

  public Command setAngleCommand(double setpointRads) {
    return run(() -> setSetpoint(setpointRads))
        .until(
            () ->
                Util.epsilonEquals(
                    getPositionRadians(),
                    setpointRads,
                    Constants.Wrist.WRIST_TOLERANCE.getRadians()));
  }

  public Command setExtensionCommand(double setpointRads, double thresholdRads) {
    return run(() -> setSetpoint(setpointRads)).until(() -> getPositionRadians() >= thresholdRads);
  }

  public Command setTuckCommand(double setpointRads, double thresholdRads) {
    return run(() -> setSetpoint(setpointRads)).until(() -> getPositionRadians() <= thresholdRads);
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
