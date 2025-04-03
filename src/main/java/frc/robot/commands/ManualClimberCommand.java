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
    double joystick = -operator.getLeftY();
    joystick = MathUtil.applyDeadband(joystick, 0.1);

    joystick = Math.pow(joystick, 2) * Math.signum(joystick);
    if (joystick == 0) { // If there isn't any input, maintain the position
      climber.runSetpoint(lastClimberPose);
    } else if (Math.abs(joystick) >= 0.075) {
      climber.runDutyCycle(joystick);
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
