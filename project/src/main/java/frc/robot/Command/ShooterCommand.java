
package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Transfer.TransferConstants;

public class ShooterCommand extends Command {
  public ShooterCommand() {
    addRequirements(Shooter.getInstance(), Transfer.getInstance());
  }

  @Override
  public void initialize() {
    RobotContainer.currentAction.addString("current ACtion", "shooter");
  }

  @Override
  public void execute() {
    Shooter.getInstance().setVelocity(5000);
    if((Shooter.getInstance().getRightVelocity()+Shooter.getInstance().getLeftVelocity())/2 > 5000) {
      Transfer.getInstance().setVoltage(TransferConstants.INTAKE_VOLTAGE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.currentAction.addString("current ACtion", "finished shooter");
    Shooter.getInstance().setVelocity(0);
    Transfer.getInstance().setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return !Transfer.getInstance().isFirstSensor() && !Transfer.getInstance().isSeconSensor();
  }
}
