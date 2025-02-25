package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.Climber;

public class ClimberTestCommand extends Command {

  Climber climber;
  CommandXboxController controller;

  public ClimberTestCommand(Climber climber, CommandXboxController controller) {
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    double percent = -controller.getLeftY();
    percent = Math.abs(percent) < 0.1 ? 0 : (percent - 0.1) / 0.9;
    if (percent == 0) climber.stop();
    else climber.runDutyCycle(percent);
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
