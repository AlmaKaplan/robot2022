
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

public class IntakeCommand extends Command {
  public IntakeCommand() {
    addRequirements(IntakeRollers.getInstance(), IntakeArm.getInstance(), Transfer.getInstance(),Shooter.getInstance());
  }

  @Override
  public void initialize() {
    CurrentAction.currentAction.addString("current Action", "Intake");
    System.out.println("start");
  }

  @Override
  public void execute() {
    IntakeRollers.getInstance().setVoltage(IntakeRollersConstants.INTAKE_VOLTAGE);
    IntakeArm.getInstance().setControl(IntakeArmConstants.OPEN);
    Transfer.getInstance().setVoltage(TransferConstants.INTAKE_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    CurrentAction.currentAction.addString("current Action", "finished Intake");
    IntakeRollers.getInstance().setVoltage(0);
    IntakeArm.getInstance().setControl(IntakeArmConstants.CLOSE);
    Transfer.getInstance().setVoltage(0);
    Shooter.getInstance().setVoltage(10);
  }

  @Override
  public boolean isFinished() {
    return Transfer.getInstance().isTwoSensors();
  }
}
