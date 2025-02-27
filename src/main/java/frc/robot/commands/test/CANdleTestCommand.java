package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.candle.candle;

public class CANdleTestCommand extends Command {

  candle candleCommand;

  public void CANdleTestCommand(int state) {

    candleCommand = this.candleCommand;
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
