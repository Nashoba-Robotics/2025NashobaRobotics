package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuneWristCommand extends Command {
  Wrist wrist;
  double lastkV, lastkP, lastkD;
  LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Wrist/kV", 0.0);
  LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Wrist/kP", 0.0);
  LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Wrist/kD", 0.0);
  LoggedNetworkNumber positionRad = new LoggedNetworkNumber("/Tuning/Wrist/setpointRads", 0.0);

  public TuneWristCommand(Wrist wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    lastkV = Constants.Elevator.PID.kV;
    lastkP = Constants.Elevator.PID.kP;
    lastkD = Constants.Elevator.PID.kD;
  }

  @Override
  public void execute() {
    if (lastkV != kV.get() || lastkP != kP.get() || lastkD != kD.get()) {

      wrist.setPID(kV.get(), kP.get(), kD.get());
      lastkV = kV.get();
      lastkP = kP.get();
      lastkD = kD.get();
    }

    wrist.runSetpoint(positionRad.get());
  }

  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
