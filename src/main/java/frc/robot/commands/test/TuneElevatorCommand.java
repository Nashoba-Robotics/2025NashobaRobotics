package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TuneElevatorCommand extends Command {
  Elevator elevator;
  double lastkV, lastkP, lastkD;
  LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Elevator/kV", 0.0);
  LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Elevator/kP", 0.0);
  LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Elevator/kD", 0.0);
  LoggedNetworkNumber ExtensionMeters =
      new LoggedNetworkNumber("/Tuning/Elevator/ExtensionMeters", 0.0);

  public TuneElevatorCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
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

      elevator.setPID(kV.get(), kP.get(), kD.get());
      lastkV = kV.get();
      lastkP = kP.get();
      lastkD = kD.get();
    }

    elevator.setExtension(ExtensionMeters.get());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
