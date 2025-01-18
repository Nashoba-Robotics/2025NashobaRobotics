package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  public Manipulator() {
    io = new ManipulatorIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  public void setSetpoint(double setpointRadsPerSec) {
    io.setSetpoint(setpointRadsPerSec);
  }

  public void setPID(double kP, double kD, double kS, double kV) {
    io.setkP(kP);
    io.setkD(kD);
    io.setkS(kS);
    io.setkV(kV);
  }
}
