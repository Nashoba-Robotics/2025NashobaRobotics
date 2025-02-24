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

  public void stop() {
    io.stop();
  }

  public boolean isCoralPresent() {
    return inputs.coralPresent;
  }

  public Command coralIntakeCommand() {
    return new ConditionalCommand(
        Commands.none(),
        run(() -> runPercentOutput(0.3)).until(() -> isCoralPresent()).finallyDo(() -> stop()),
        () -> isCoralPresent());
  }

  public Command algaeIntakeCommand() {
    return run(() -> runPercentOutput(-1))
        .raceWith(new SuppliedWaitCommand(() -> 0.15))
        .andThen(
            Commands.waitUntil(() -> inputs.velocityRadPerSec >= -20),
            new SuppliedWaitCommand(() -> 0.2))
        .finallyDo(() -> runPercentOutput(-0.4));
  }

  public Command ejectCommand() {
    return run(() -> runPercentOutput(0.8))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> stop());
  }

  public Command L4ejectCommand() {
    return run(() -> runPercentOutput(-0.8))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> stop());
  }

  public Command toNeutralAid() {
    return runOnce(() -> runPercentOutput(-0.05));
  }

  public Command stopCommand() {
    return runOnce(() -> stop());
  }
}
