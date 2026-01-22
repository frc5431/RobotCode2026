package frc.robot.subsystems.intake.roller;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerIOConstants.RollerIOModes;

public class Roller extends SubsystemBase {

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private RollerIOModes mode;

  public Roller(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
    this.mode = RollerIOModes.IDLE;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(inputs);
  }

  public void runEnum(RollerIOModes rollerIOModes, boolean useRPM) {
    this.mode = rollerIOModes;
    if (useRPM) {
      rollerIO.setRPM(rollerIOModes.speed.magnitude());
    }
    else {
      rollerIO.setRollerVoltage(rollerIOModes.voltage);
    }
  }
  
  public Command runIntakeCommand(RollerIOModes rollerIOModes, boolean useRPM) {
    return new RunCommand(() -> this.runEnum(rollerIOModes, useRPM), this)
      .withName("Intake.runEnum");
  }
}
