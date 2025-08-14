
package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CurrentAction;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.IntakeArm.IntakeArm;
import frc.robot.Subsystems.IntakeArm.IntakeArmConstants;
import frc.robot.Subsystems.IntakeRollers.IntakeRollers;
import frc.robot.Subsystems.IntakeRollers.IntakeRollersConstants;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Transfer.Transfer;
import frc.robot.Subsystems.Transfer.TransferConstants;

public class IntakeAndShooter extends Command {
  public IntakeAndShooter() {
    addRequirements(IntakeRollers.getInstance(), IntakeArm.getInstance(), Transfer.getInstance(), Shooter.getInstance());
  }

  @Override
  public void initialize() {
    CurrentAction.currentAction.addString("current Action", "Intake and shooter");
  }

  @Override
  public void execute() {
    IntakeArm.getInstance().setControl(IntakeArmConstants.OPEN);
    Shooter.getInstance().setVelocity(5000);
    if((Shooter.getInstance().getRightVelocity()+Shooter.getInstance().getLeftVelocity())/2 > 5000) {
      Transfer.getInstance().setVoltage(TransferConstants.INTAKE_VOLTAGE);
    }
    if (Transfer.getInstance().isFirstSensor()||Transfer.getInstance().isSeconSensor()) {
      IntakeRollers.getInstance().setVoltage(IntakeRollersConstants.INTAKE_VOLTAGE);
    } else {
      IntakeRollers.getInstance().setVoltage(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    CurrentAction.currentAction.addString("current Action", "fnished Intake and shooter");
    IntakeRollers.getInstance().setVoltage(0);
    IntakeArm.getInstance().setControl(IntakeArmConstants.CLOSE);
    Transfer.getInstance().setVoltage(0);
    Shooter.getInstance().setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
