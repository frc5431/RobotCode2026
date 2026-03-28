package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetRollerConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;

public class Carpet extends SubsystemBase {
  private final CarpetIO carpetIO;
  private final CarpetIOInputsAutoLogged carpetInputs = new CarpetIOInputsAutoLogged();

  private CarpetModes mode;

  public Carpet(CarpetIO carpetIO) {
    this.carpetIO = carpetIO;
    this.mode = CarpetModes.IDLE;
  }

  @Override
  public void periodic() {
    carpetIO.updateInputs(carpetInputs);
    Logger.processInputs("Carpet/", carpetInputs);

    Logger.recordOutput("Carpet/Mode", mode);
  }

  public void runRollerEnum(CarpetModes carpetMode) {
    this.mode = carpetMode;
      carpetIO.setRollerVoltage(mode.voltage.baseUnitMagnitude());
  }

  public Command runCarpetCommand(CarpetModes carpetMode) {
    return new RunCommand(() -> {
      this.runRollerEnum(carpetMode);
    }, this).withName("Carpet.runCarpetEnum" + carpetMode.toString());
  }

  public Command tuneCarpet(){
  return new RunCommand(() -> {
      carpetIO.setRPM(AngularVelocity.ofRelativeUnits(CarpetRollerConstants.tuneDesiredSpeed.get(), RPM));
    }, this);
  }

  public Command runCarpetRPM(AngularVelocity RPM) {
    return new RunCommand(() -> {
      carpetIO.setRPM(RPM);
    }, this);
  }

}
