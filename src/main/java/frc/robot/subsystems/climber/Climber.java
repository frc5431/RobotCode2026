
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberModes;

public class Climber extends SubsystemBase {
    private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged carpetInputs = new ClimberIOInputsAutoLogged();

  private ClimberModes mode;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
    this.mode = ClimberModes.STOW;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(carpetInputs);
    Logger.processInputs("Climber/", carpetInputs);

    Logger.recordOutput("Climber/Mode", mode);
  }

  public void runClimberEnum(ClimberModes climberMode) {
    this.mode = climberMode;
    climberIO.setVoltage(climberMode.positionAngle.in(Rotations));
    // climberIO.setClimberPosition(mode.positionAngle.baseUnitMagnitude());
  }

  public Command runClimberCommand(ClimberModes climberModes) {
    return new RunCommand(() -> {
      this.runClimberEnum(climberModes);
    }, this).withName("Climber.runClimberEnum");
  }

}
