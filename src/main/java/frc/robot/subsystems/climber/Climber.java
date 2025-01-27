package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber() {
    io = new ClimberIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setPivotSetpoint(double setpointRads) {
    io.setPivotSetpoint(setpointRads);
  }

  public void setGrabberVoltage(double setpointVoltage) {
    io.setGrabberVoltage(setpointVoltage);
  }

  public void zeroClimber() {
    io.setPivotPosition(0);
  }

  public void setPID(double kV, double kP, double kD) {
    io.setkV(kV);
    io.setkP(kP);
    io.setkD(kD);
  }
}
