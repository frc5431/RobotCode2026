package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase {
  private final RollerIO rollerIO;
  private final PivotIO pivotIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  
  private IntakeMode mode;
  

  public Intake(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.mode = IntakeMode.STOW;
  }
  
  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);
    
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);

    Logger.recordOutput("Intake/Mode", mode);
  }

  public void runRollerEnum(IntakeMode intakeMode) {
    this.mode = intakeMode;
      rollerIO.setRollerVoltage(mode.voltage.baseUnitMagnitude());
  }

  public void runPivotEnum(IntakeMode intakeMode) {
    this.mode = intakeMode;
    pivotIO.setPosition(mode.position.magnitude());
  }

  public Command runIntakeCommand(IntakeMode intakeMode) {
    return new RunCommand(() -> {
      this.runRollerEnum(intakeMode);
      this.runPivotEnum(intakeMode);
    }, this).withName("Intake.runIntakeEnum");
  }
}