package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureGoal {
    NEUTRAL(0, 0.05),
    INTAKE(0, 0.05),

    L4CORAL(1.3, -3.4),
    L3CORAL(1.025, 0.05),
    L2CORAL(0.635, 0.05),
    L1CORAL(0.4, 0.3),

    BARGEALGAE(1.4, 2.5),
    L3ALGAE(0.85, 0.75),
    L2ALGAE(0.45, 0.75),
    PROCESSORALGAE(0.15, 0.);

    private double extensionMeters;
    private double angleRads;

    SuperstructureGoal(double extensionMeters, double angleRads) {
      this.extensionMeters = extensionMeters;
      this.angleRads = angleRads;
    }
  }

  private SuperstructureGoal goal = SuperstructureGoal.NEUTRAL;

  private final Elevator elevator;
  private final Wrist wrist;
  private final Manipulator manipulator;
  private final Hopper hopper;
  private final Climber climber;

  private final Trigger score = RobotContainer.driver.rightTrigger(0.65);

  public Superstructure(
      Elevator elevator, Wrist wrist, Manipulator manipulator, Hopper hopper, Climber climber) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.manipulator = manipulator;
    this.hopper = hopper;
    this.climber = climber;
  }

  public Command setNeutral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.NEUTRAL),
        new ParallelCommandGroup(
            elevator.runNeutralPrep(), wrist.runAngleCommand(Presets.NEUTRAL.angleRads)),
        elevator.runSetpointCommand(Presets.NEUTRAL.extensionMeters));
  }

  public Command setIntake() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.INTAKE),
        new ParallelCommandGroup(
            elevator.runNeutralPrep(), wrist.runAngleCommand(Presets.INTAKE.angleRads)),
        elevator.runSetpointCommand(Presets.INTAKE.extensionMeters),
        new ParallelCommandGroup(
            hopper.intakeCommand().until(() -> hasCoral()), manipulator.coralIntakeCommand()));
  }

  public Command setL4Coral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L4CORAL),
        elevator.runExtensionCommand(Presets.L4CORAL.extensionMeters, 0.275),
        wrist.runAngleCommand(Presets.L4CORAL.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.L4ejectCommand());
  }

  public Command setL3Coral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L3CORAL),
        elevator.runExtensionCommand(Presets.L3CORAL.extensionMeters, 0.275),
        wrist.runAngleCommand(Presets.L3CORAL.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setL2Coral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L2CORAL),
        elevator.runExtensionCommand(Presets.L2CORAL.extensionMeters, 0.275),
        wrist.runAngleCommand(Presets.L2CORAL.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setL1Coral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L1CORAL),
        elevator.runExtensionCommand(Presets.L1CORAL.extensionMeters, 0.05),
        wrist.runAngleCommand(Presets.L1CORAL.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.L1ejectCommand());
  }

  public Command setBargeAlgae() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.BARGEALGAE),
        elevator.runNeutralPrep(),
        wrist.runAngleCommand(Presets.BARGEALGAE.angleRads),
        elevator.runSetpointCommand(Presets.BARGEALGAE.extensionMeters),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setL3Algae() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L3ALGAE),
        elevator.runExtensionCommand(Presets.L3ALGAE.extensionMeters, 0.15),
        wrist.runAngleCommand(Presets.L3ALGAE.angleRads),
        manipulator.algaeIntakeCommand());
  }

  public Command setL2Algae() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L2ALGAE),
        elevator.runExtensionCommand(Presets.L2ALGAE.extensionMeters, 0.15),
        wrist.runAngleCommand(Presets.L2ALGAE.angleRads),
        manipulator.algaeIntakeCommand());
  }

  public Command setProcessor() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.PROCESSORALGAE),
        elevator.runSetpointCommand(Presets.PROCESSORALGAE.extensionMeters),
        wrist.runAngleCommand(Presets.PROCESSORALGAE.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command autoSetL4Coral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L4CORAL),
        elevator.runExtensionCommand(Presets.L4CORAL.extensionMeters, 0.275),
        wrist.runAngleCommand(Presets.L4CORAL.angleRads));
  }

  public Command autoSetL2Coral() {
    return new SequentialCommandGroup(
        runOnce(() -> goal = SuperstructureGoal.L2CORAL),
        elevator.runExtensionCommand(Presets.L2CORAL.extensionMeters, 0.275),
        wrist.runAngleCommand(Presets.L2CORAL.angleRads));
  }

  public Command autoIntake() {
    return new ParallelCommandGroup(manipulator.autoIntakeCommand(), hopper.intakeCommand());
  }

  public boolean hasCoral() {
    return manipulator.isCoralPresent();
  }

  public SuperstructureGoal getGoal() {
    return goal;
  }
}
