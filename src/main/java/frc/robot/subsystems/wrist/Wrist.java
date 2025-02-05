package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist() {
    io = new WristIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setSetpoint(double setpointRads) {
    io.setSetpoint(setpointRads);
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
