package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.Climber;

public class ManualClimberCommand extends Command {
  CommandXboxController operator;

  Climber climber;
  double lastClimberPose;

  public ManualClimberCommand(Climber climber, CommandXboxController operator) {
    this.operator = operator;
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    lastClimberPose = climber.getAngleRads();
  }

  @Override
  public void execute() {
    double leftDownTrigger = -operator.getLeftTriggerAxis();
    double rightUpTrigger = operator.getRightTriggerAxis();
    MathUtil.applyDeadband(leftDownTrigger, 0.1);
    MathUtil.applyDeadband(rightUpTrigger, 0.1);

    leftDownTrigger = Math.pow(leftDownTrigger, 2) * Math.signum(leftDownTrigger);
    rightUpTrigger = Math.pow(rightUpTrigger, 2) * Math.signum(rightUpTrigger);
    if (leftDownTrigger == 0
        && rightUpTrigger == 0) { // If there isn't any input, maintain the position
      climber.runSetpoint(lastClimberPose);
    } else if (leftDownTrigger >= 0.1) {
      climber.runDutyCycle(leftDownTrigger);
      lastClimberPose = climber.getAngleRads();
    } else {
      climber.runDutyCycle(rightUpTrigger);
      lastClimberPose = climber.getAngleRads();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
