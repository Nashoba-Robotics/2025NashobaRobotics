package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command intakeCommand() {
    return run(() -> runPercentOutput(-0.8))
        .raceWith(new SuppliedWaitCommand(() -> 0.200))
        .andThen(
            Commands.waitUntil(() -> inputs.velocityRadPerSec > -375.0),
            new SuppliedWaitCommand(() -> 0.050))
        .finallyDo(() -> runPercentOutput(-0.05));
  }

  public Command ejectCommand() {
    return run(() -> runPercentOutput(1.0))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> runPercentOutput(0));
  }

  public Command L1ejectCommand() {
    return run(() -> runPercentOutput(0.75))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> runPercentOutput(0));
  }
}
