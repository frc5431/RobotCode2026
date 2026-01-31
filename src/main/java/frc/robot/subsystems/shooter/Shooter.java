package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
  private final AnglerIO anglerIO;
  private final FlywheelIO flywheelIO;

  private final AnglerIOInputsAutoLogged anglerInputs = new AnglerIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  
  private IntakeMode mode;
  

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

  public void runFlywheelEnum(IntakeMode intakeMode) {
    this.mode = intakeMode;
      flywheelIO.setRPM(mode.voltage.baseUnitMagnitude());
  }

  public void runAnglerEnum(IntakeMode intakeMode) {
    this.mode = intakeMode;
    anglerIO.setPosition(mode.position.magnitude());
  }

  // public Command runIntakeCommand(IntakeMode intakeMode) {
  //   return new RunCommand(() -> {
  //     this.runFlywheelEnum(intakeMode);
  //     this.runAnglerEnum(intakeMode);
  //   }, this).withName("Shooter.runIntakeEnum");
  // }
}
