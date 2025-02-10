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
    NEUTRAL(0, 0),
    INTAKE(0, -0.55),

    L4CORALPREP(1.2, Math.PI / 2),
    L4CORAL(1.2, 2.15),
    L3CORAL(0.52, 2.5),
    L2CORAL(0.13, 2.5),
    L1CORAL(0, 0),

    BARGEALGAE(0, 0),
    L3ALGAE(0, 0),
    L2ALGAE(0.05, 3.55),
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
