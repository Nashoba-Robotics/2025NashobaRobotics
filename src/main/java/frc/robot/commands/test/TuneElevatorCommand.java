package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class TuneElevatorCommand extends Command {
  Elevator elevator;
  double lastkV, lastkP, lastkD;

  public TuneElevatorCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    lastkV = Constants.Elevator.PID.kV;
    lastkP = Constants.Elevator.PID.kP;
    lastkD = Constants.Elevator.PID.kD;

    SmartDashboard.putNumber("kV", lastkV);
    SmartDashboard.putNumber("kP", lastkP);
    SmartDashboard.putNumber("kD", lastkD);

    SmartDashboard.putNumber("Elevator Extension(meters)", 0);
  }

  @Override
  public void execute() {
    double kV = SmartDashboard.getNumber("kV", 0);
    double kP = SmartDashboard.getNumber("kP", 0);
    double kD = SmartDashboard.getNumber("kD", 0);
    if (kV != lastkV || kP != lastkP || kD != lastkD) {
      lastkV = kV;
      lastkP = kP;
      lastkD = kD;
      elevator.setPID(kV, kP, kD);
    }

    double extension = SmartDashboard.getNumber("Elevator Extension(meters)", 0);
    elevator.setSetpoint(extension);
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
