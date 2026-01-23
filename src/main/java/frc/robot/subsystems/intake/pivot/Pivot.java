package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants.IntakePivotPositions;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private IntakePivotPositions mode;

  public Pivot(PivotIO PivotIO) {
    this.pivotIO = PivotIO;
    this.mode = IntakePivotPositions.STOW;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);
    Logger.recordOutput("Intake/Pivot/Mode", mode);
  }

  public void runEnum(IntakePivotPositions pivotPosition) {
    this.mode = pivotPosition;
    pivotIO.setPosition(pivotPosition.position.magnitude());
  }

  public Command runPivotCommand(IntakePivotPositions pivotPosition) {
    return new RunCommand(() -> this.runEnum(pivotPosition), this).withName("IntakePivot.runEnum");
  }
}
