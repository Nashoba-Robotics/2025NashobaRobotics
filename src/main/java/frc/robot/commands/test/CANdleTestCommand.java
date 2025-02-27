package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.candle.Candle;

public class CANdleTestCommand extends Command {

  Candle candleCommand;

  public CANdleTestCommand(int state) {

    this.candleCommand = candleCommand;
  }

  @Override
  public void execute() {
    candleCommand.setLarsonFront();
  }

  @Override
  public void end(boolean interrupted) {
    candleCommand.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
