package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureGoal {
    NEUTRAL(0, 0),
    INTAKE(0, 0),

    L4CORALPREP(0, 0),
    L4CORAL(0, 0),
    L3CORAL(0, 0),
    L2CORAL(0, 0),
    L1CORAL(0, 0),

    BARGEALGAE(0, 0),
    L3ALGAE(0, 0),
    L2ALGAE(0, 0),
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
  private final Manipulator manipulator;
  private final Hopper hopper;
  private final Climber climber;

  private final CommandXboxController driver = RobotContainer.driver;

  public Superstructure(Elevator elevator, Wrist wrist, Manipulator manipulator, Hopper hopper, Climber climber) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.manipulator = manipulator;
    this.hopper = hopper;
    this.climber = climber;
  }

  public Command setNeutral() {
    goal = SuperstructureGoal.NEUTRAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(goal.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters)),
        new ParallelCommandGroup(
            wrist.runAngleCommand(goal.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters)),
        () -> elevator.isNearL4());
  }

  public Command setIntake() {
    goal = SuperstructureGoal.INTAKE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(goal.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            new ParallelCommandGroup(hopper.intakeCommand(), manipulator.intakeCommand())),
        new ParallelCommandGroup(
            wrist.runAngleCommand(goal.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            hopper.intakeCommand(),
            manipulator.intakeCommand()),
        () -> elevator.isNearL4());
  }

  public Command setL4Coral() {
    goal = SuperstructureGoal.L4CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
            manipulator.L4ejectCommand()),
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
                elevator.runSetpointCommand(goal.extensionMeters)),
            wrist.runAngleCommand(goal.angleRads),
            new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
            manipulator.L4ejectCommand()),
        () -> elevator.isNearL3());
  }

  public Command setL3Coral() {
    goal = SuperstructureGoal.L3CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
            manipulator.ejectCommand()),
        new ParallelCommandGroup(
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
                manipulator.ejectCommand())),
        () -> elevator.isNearL4());
  }

  public Command setL2Coral() {
    goal = SuperstructureGoal.L2CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
            manipulator.ejectCommand()),
        new ParallelCommandGroup(
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
                manipulator.ejectCommand())),
        () -> elevator.isNearL4());
  }

  public Command setL1Coral() {
    goal = SuperstructureGoal.L1CORAL;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
            manipulator.ejectCommand()),
        new ParallelCommandGroup(
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> driver.rightTrigger(0.6).getAsBoolean()),
                manipulator.ejectCommand())),
        () -> elevator.isNearL4());
  }

  public Command setBargeAlgae() {
    goal = SuperstructureGoal.BARGEALGAE;
    return new ParallelCommandGroup(
        wrist.runAngleCommand(goal.angleRads), elevator.runSetpointCommand(goal.extensionMeters));
  }

  public Command setL3Algae() {
    goal = SuperstructureGoal.L3ALGAE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }

  public Command setL2Algae() {
    goal = SuperstructureGoal.L2ALGAE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }

  public Command setProcessor() {
    goal = SuperstructureGoal.PROCESSORALGAE;
    return new ConditionalCommand(
        new SequentialCommandGroup(
            wrist.runAngleCommand(SuperstructureGoal.L4CORALPREP.angleRads),
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads)),
        new ParallelCommandGroup(
            elevator.runSetpointCommand(goal.extensionMeters),
            wrist.runAngleCommand(goal.angleRads)),
        () -> elevator.isNearL4());
  }
}
