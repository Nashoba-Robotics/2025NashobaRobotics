package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.Climber;

public class ClimberTestDownCommand extends Command {

  Climber climber;
  CommandXboxController controller;

  double lastClimberPose;

  public ClimberTestDownCommand(Climber climber, CommandXboxController controller) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    double percent = -controller.getLeftTriggerAxis();
    percent = MathUtil.applyDeadband(percent, 0.1);
    percent = Math.pow(percent, 2) * Math.signum(percent);
    if (percent == 0) climber.runSetpoint(lastClimberPose);
    else {
      climber.runDutyCycle(percent);
      lastClimberPose = climber.getAngleRads();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.runSetpoint(lastClimberPose);
    ;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
