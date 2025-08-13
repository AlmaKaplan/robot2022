
package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrain.DriveTrain;

public class DriveTrainCommand extends Command {
  private Supplier<Double> leftJoystickPosition;
  private Supplier<Double> rightJoystickPosition;

  public DriveTrainCommand(Supplier<Double> leftJoystickPosition, Supplier<Double> rightJoystickPosition) {
    addRequirements(DriveTrain.getInstance());
    this.leftJoystickPosition = leftJoystickPosition;
    this.rightJoystickPosition = rightJoystickPosition;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DriveTrain.getInstance().leftControl(leftJoystickPosition.get()*-12);
    DriveTrain.getInstance().rightControl(rightJoystickPosition.get()*-12);
  }

  @Override
  public void end(boolean interrupted) {
    DriveTrain.getInstance().leftControl(0);
    DriveTrain.getInstance().rightControl(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
