package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class ManualExtensionCommand extends Command {
  CommandXboxController operator;

  Elevator elevator;
  Wrist wrist;
  double lastWristPose;
  double lastElevatorPose;

  public ManualExtensionCommand(CommandXboxController operator, Elevator elevator, Wrist wrist) {
    this.operator = operator;
    this.elevator = elevator;
    this.wrist = wrist;
    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {
    lastWristPose = wrist.getAngleRads();
    lastElevatorPose = elevator.getPositionMeters();
  }

  @Override
  public void execute() {
    double lefty = -operator.getLeftY();
    double righty = -operator.getRightY();
    lefty = Math.abs(lefty) < 0.1 ? 0 : (lefty - 0.1) / 0.9; // Put deadzone in Constants
    righty = Math.abs(righty) < 0.1 ? 0 : (righty - 0.25) / 0.75;
    if (lefty == 0) { // If there isn't any input, maintain the position
      elevator.setExtension(lastElevatorPose);
    } else {
      elevator.setDutyCycle(lefty * 0.1);
      lastElevatorPose = elevator.getPositionMeters();
    }
    if (righty == 0) { // If there isn't any input, maintain the position
      wrist.setSetpoint(lastWristPose);
    } else {
      wrist.setDutyCycle(righty * 0.075 + 0.01);
      lastWristPose = wrist.getAngleRads();
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    wrist.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
