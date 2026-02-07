package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants.IntakePivotModes;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants.IntakeRollerModes;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase {
  private final RollerIO rollerIO;
  private final PivotIO pivotIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  
  private IntakeRollerModes rollerMode;
  private IntakePivotModes pivotMode;
  

  public Intake(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.rollerMode = IntakeRollerModes.IDLE;
    this.pivotMode = IntakePivotModes.STOW;
  }
  
  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);
    Logger.recordOutput("Intake/Roller/Mode", rollerMode);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
    Logger.recordOutput("Intake/Pivot/Mode", pivotMode);
  }

  public void runRollerEnum(IntakeRollerModes rollerMode) {
    this.rollerMode = rollerMode;
      rollerIO.setRollerVoltage(rollerMode.voltage.baseUnitMagnitude());
  }

  public void runPivotEnum(IntakePivotModes pivotMode) {
    this.pivotMode = pivotMode;
    pivotIO.setPosition(pivotMode.position.magnitude());
  }

  public Command runIntakeCommand(IntakePivotModes pivotMode, IntakeRollerModes rollerMode) {
    return new RunCommand(() -> {
      this.runRollerEnum(rollerMode);
      this.runPivotEnum(pivotMode);
    }, this).withName("Intake.runIntakeEnum" + rollerMode.toString());
  }
}