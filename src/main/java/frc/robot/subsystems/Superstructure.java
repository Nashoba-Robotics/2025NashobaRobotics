package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    L3CORAL(1.01, 0.05),
    L2CORAL(0.635, 0.05),
    L1CORAL(0.45, 0.05),

    BARGEALGAE(1.4, 3.2),
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
    goal = SuperstructureGoal.NEUTRAL;
    return new SequentialCommandGroup(
        new ParallelCommandGroup(elevator.runNeutralPrep(), wrist.runAngleCommand(goal.angleRads)),
        elevator.runSetpointCommand(goal.extensionMeters));
  }

  public Command setIntake() {
    goal = SuperstructureGoal.INTAKE;
    return new SequentialCommandGroup(
        new ParallelCommandGroup(elevator.runNeutralPrep(), wrist.runAngleCommand(goal.angleRads)),
        elevator.runSetpointCommand(goal.extensionMeters),
        new ParallelCommandGroup(hopper.intakeCommand(), manipulator.coralIntakeCommand()));
  }

  public Command setL4Coral() {
    goal = SuperstructureGoal.L4CORAL;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.275),
        wrist.runAngleCommand(goal.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.L4ejectCommand());
  }

  public Command setL3Coral() {
    goal = SuperstructureGoal.L3CORAL;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.275),
        wrist.runAngleCommand(goal.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setL2Coral() {
    goal = SuperstructureGoal.L2CORAL;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.275),
        wrist.runAngleCommand(goal.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setL1Coral() {
    goal = SuperstructureGoal.L1CORAL;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.275),
        wrist.runAngleCommand(goal.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setBargeAlgae() {
    goal = SuperstructureGoal.BARGEALGAE;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.275),
        wrist.runAngleCommand(goal.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }

  public Command setL3Algae() {
    goal = SuperstructureGoal.L3ALGAE;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.15),
        wrist.runAngleCommand(goal.angleRads),
        manipulator.algaeIntakeCommand());
  }

  public Command setL2Algae() {
    goal = SuperstructureGoal.L2ALGAE;
    return new SequentialCommandGroup(
        elevator.runExtensionCommand(goal.extensionMeters, 0.15),
        wrist.runAngleCommand(goal.angleRads),
        manipulator.algaeIntakeCommand());
  }

  public Command setProcessor() {
    goal = SuperstructureGoal.PROCESSORALGAE;
    return new SequentialCommandGroup(
        elevator.runSetpointCommand(goal.extensionMeters),
        wrist.runAngleCommand(goal.angleRads),
        new WaitUntilCommand(() -> score.getAsBoolean()),
        manipulator.ejectCommand());
  }
}
