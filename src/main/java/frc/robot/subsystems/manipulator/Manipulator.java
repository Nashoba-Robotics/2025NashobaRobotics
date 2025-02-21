package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SuppliedWaitCommand;
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

  public void runPercentOutput(double setpointPercent) {
    io.runPercentOutput(setpointPercent);
  }

  public boolean isCoralPresent(){
    return inputs.coralPresent;
  }

  public Command intakeCommand() {
    return new ConditionalCommand(
      Commands.none(),
      run(() -> runPercentOutput(0.8)).until(() -> isCoralPresent()).finallyDo(() ->runPercentOutput(0.0)),
      () -> isCoralPresent());
  }

  public Command ejectCommand() {
    return run(() -> runPercentOutput(0.8))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> runPercentOutput(0.0));
  }

  public Command L4ejectCommand() {
    return run(() -> runPercentOutput(-0.8))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> runPercentOutput(0.0));
  }
}
