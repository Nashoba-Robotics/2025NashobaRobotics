package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureGoal {
    NEUTRAL(0, Math.PI / 2),
    INTAKE(0, -0.45),

    L4CORALPREP(1.25, Math.PI / 2),
    L4CORAL(1.25, 2.15),
    L3CORAL(0.515, 2.6),
    L2CORAL(0.115, 2.6),
    L1CORAL(0, 0.1),

    BARGEALGAE(1.34, 3.95),
    L3ALGAE(0.4, 3.4),
    L2ALGAE(0.05, 3.4),
    PROCESSORALGAE(0, 0);

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

  public Superstructure(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public Command setNeutral() {
    goal = SuperstructureGoal.NEUTRAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(goal.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters)),
        new ParallelCommandGroup(
            wrist.setAngleCommand(goal.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters)),
        () -> elevator.isNearL4());
  }

  public Command setIntake() {
    goal = SuperstructureGoal.INTAKE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(goal.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters)),
        new ParallelCommandGroup(
            wrist.setAngleCommand(goal.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters)),
        () -> elevator.isNearL4());
  }

  public Command scoreL4Coral() {
    goal = SuperstructureGoal.L4CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
                elevator.setSetpointCommand(goal.extensionMeters)),
            wrist.setAngleCommand(goal.angleRads)),
        () -> elevator.isNearL3());
  }

  public Command scoreL3Coral() {
    goal = SuperstructureGoal.L3CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }

  public Command scoreL2Coral() {
    goal = SuperstructureGoal.L2CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }

  public Command scoreL1Coral() {
    goal = SuperstructureGoal.L1CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }

  public Command setBargeAlgae() {
    goal = SuperstructureGoal.BARGEALGAE;
    return new ParallelCommandGroup(
        wrist.setAngleCommand(goal.angleRads), elevator.setSetpointCommand(goal.extensionMeters));
  }

  public Command setL3Algae() {
    goal = SuperstructureGoal.L3ALGAE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }

  public Command setL2Algae() {
    goal = SuperstructureGoal.L2ALGAE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.setAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.setSetpointCommand(goal.extensionMeters),
            wrist.setAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }
}
