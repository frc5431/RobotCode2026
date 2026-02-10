package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
  private final AnglerIO anglerIO;
  private final FlywheelIO flywheelIO;

  private final AnglerIOInputsAutoLogged anglerInputs = new AnglerIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
    

  public Shooter(AnglerIO anglerIO, FlywheelIO flywheelIO) {
    this.anglerIO = anglerIO;
    this.flywheelIO = flywheelIO;
  }
  
  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerInputs);
    Logger.processInputs("Shooter/Angler", anglerInputs);
    
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    // Logger.recordOutput("Intake/Mode", mode);
  }

  // public void runFlywheelEnum(IntakeMode intakeMode) {
  //   this.mode = intakeMode;
  //     flywheelIO.setRPM(mode.voltage.baseUnitMagnitude());
  // }

  // public void runAnglerEnum(IntakeMode intakeMode) {
  //   this.mode = intakeMode;
  //   anglerIO.setPosition(mode.position.magnitude());
  // }

  // public Command runIntakeCommand(IntakeMode intakeMode) {
  //   return new RunCommand(() -> {
  //     this.runFlywheelEnum(intakeMode);
  //     this.runAnglerEnum(intakeMode);
  //   }, this).withName("Shooter.runIntakeEnum");
  // }
}
