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
  
  private IntakeMode intakeMode;
  // private IntakePivotModes pivotMode;
  

  public Intake(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.intakeMode = IntakeMode.STOW;
  }
  
  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
    
    Logger.recordOutput("Intake/Mode", intakeMode);
  }

  public void runIntakeEnum(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
    rollerIO.setRollerVoltage(intakeMode.voltage.baseUnitMagnitude());
    pivotIO.setPosition(intakeMode.position.magnitude());
  }

  public Command runIntakeeCommand(IntakeMode intakeMode) {
    return new RunCommand(() -> {
      this.runIntakeEnum(intakeMode);
    }, this).withName("Intake.runIntakeeCommand" + intakeMode.toString());
  }

  // public Command runPivotCommand(IntakeMode intakeMode) {
  //   return new RunCommand(() -> {
  //     this.runPivotEnum(intakeMode);
  //   }, this).withName("Intake.runPivotCommand" + intakeMode.toString());
  // }

  // public Command runRollerCommand(IntakeMode rollerMode) {
  //   return new RunCommand(() -> {
  //     this.runRollerEnum(rollerMode);
  //   }, this).withName("Intake.runRollerCommand" + rollerMode.toString());
  // }
}