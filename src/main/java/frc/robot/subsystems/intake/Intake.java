package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;

public class Intake extends SubsystemBase {

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  public Intake(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
  }
}
