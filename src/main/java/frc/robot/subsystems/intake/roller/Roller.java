package frc.robot.subsystems.intake.roller;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants.IntakeRollerModes;

public class Roller extends SubsystemBase {

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private IntakeRollerModes mode;

  public Roller(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
    this.mode = IntakeRollerModes.IDLE;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.recordOutput("Intake/Roller/Mode", mode.toString());
    Logger.processInputs("Intake/Roller", inputs);
  }

  public void runEnum(IntakeRollerModes rollerIOModes, boolean useRPM) {
    this.mode = rollerIOModes;
    if (useRPM) {
      rollerIO.setRPM(rollerIOModes.speed.magnitude());
    }
    else {
      rollerIO.setRollerVoltage(rollerIOModes.voltage);
    }
  }
  
  public Command runIntakeCommand(IntakeRollerModes rollerIOModes, boolean useRPM) {
    return new RunCommand(() -> this.runEnum(rollerIOModes, useRPM), this)
      .withName("Intake.runEnum");
  }
}
