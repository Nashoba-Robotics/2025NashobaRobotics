package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuneClimberCommand extends Command {
  Climber climber;
  double lastkV, lastkP, lastkD;
  LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Climber/kV", 0.0);
  LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Climber/kP", 0.0);
  LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Climber/kD", 0.0);
  LoggedNetworkNumber positionRad = new LoggedNetworkNumber("/Tuning/Climber/setpointRads", 0.0);

  public TuneClimberCommand(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    lastkV = Constants.Climber.PID.kV;
    lastkP = Constants.Climber.PID.kP;
    lastkD = Constants.Climber.PID.kD;
  }

  @Override
  public void execute() {
    if (lastkV != kV.get() || lastkP != kP.get() || lastkD != kD.get()) {

      climber.setPID(kV.get(), kP.get(), kD.get());
      lastkV = kV.get();
      lastkP = kP.get();
      lastkD = kD.get();
    }

    climber.runSetpoint(positionRad.get());
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
