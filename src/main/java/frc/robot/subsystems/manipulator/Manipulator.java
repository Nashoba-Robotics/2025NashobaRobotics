package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        new SequentialCommandGroup(
                run(() -> runPercentOutput(0.25)).until(() -> isCoralPresent()),
                new WaitCommand(0.0775))
            .finallyDo(() -> stop()),
        () -> isCoralPresent());
  }

  public Command autoIntakeCommand() {
    return new ConditionalCommand(
        Commands.none(),
        run(
            () -> {
              if (isCoralPresent()) stop();
              else runPercentOutput(0.35);
            }),
        () -> isCoralPresent());
  }

  public Command algaeIntakeCommand() {
    return run(() -> runPercentOutput(-0.8))
        .raceWith(new SuppliedWaitCommand(() -> 0.15))
        .andThen(
            Commands.waitUntil(() -> inputs.velocityRadPerSec >= -20),
            new SuppliedWaitCommand(() -> 0.2))
        .finallyDo(() -> runPercentOutput(-0.4));
  }

  public Command ejectCommand() {
    return run(() -> runPercentOutput(0.40))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> stop());
  }

  public Command processorEjectCommand() {
    return run(() -> runPercentOutput(0.2))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> stop());
  }

  public Command algaeEjectCommand() {
    return run(() -> runPercentOutput(1.0))
        .raceWith(new SuppliedWaitCommand(() -> 0.3))
        .finallyDo(() -> stop());
  }

  public Command L4ejectCommand() {
    return run(() -> runPercentOutput(-0.75))
        .raceWith(new SuppliedWaitCommand(() -> 0.2))
        .finallyDo(() -> stop());
  }

  public Command spitCommand() {
    return run(() -> runPercentOutput(-0.5))
        .raceWith(new SuppliedWaitCommand(() -> 0.4))
        .finallyDo(() -> stop());
  }

  public Command slowSpitCommand() {
    return run(() -> runPercentOutput(-0.15)).finallyDo(() -> stop());
  }

  public Command slowIntakeCommand() {
    return run(() -> runPercentOutput(0.15)).finallyDo(() -> stop());
  }

  public Command L1ejectCommand() {
    return run(() -> runPercentOutput(0.325))
        .raceWith(new SuppliedWaitCommand(() -> 0.9))
        .finallyDo(() -> stop());
  }

  public Command stopCommand() {
    return runOnce(() -> stop());
  }
}
