package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorDutyCycleCommand extends Command {
  Elevator elevator;
  LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Tuning/Elevator/Voltage", 0.0);

  public ElevatorDutyCycleCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setVoltage(voltage.get());
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
