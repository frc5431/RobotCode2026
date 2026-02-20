package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;

public class Shooter extends SubsystemBase {
  private final AnglerIO anglerIO;
  private final FlywheelIO flywheelIO;

  private final AnglerIOInputsAutoLogged anglerInputs = new AnglerIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  
  private ShooterModes shooterMode;

  public Shooter(AnglerIO anglerIO, FlywheelIO flywheelIO) {
    this.anglerIO = anglerIO;
    this.flywheelIO = flywheelIO;
    this.shooterMode = ShooterModes.IDLE;
  }
  
  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerInputs);
    Logger.processInputs("Shooter/Angler", anglerInputs);
    
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    Logger.recordOutput("Shooter/Mode", shooterMode);
    // System.out.println("***********************");
    // System.out.println(FlywheelIOTalonFX.plotOutput);
    // System.out.println("***********************");
  }

  public void runShooterEnum(ShooterModes mode) {
    this.shooterMode = mode;
    flywheelIO.setRPM(mode.speed.baseUnitMagnitude());
    anglerIO.setPosition(mode.angle.magnitude());
  }

  public Command runShooterCommand(ShooterModes mode) {
    return new RunCommand(() -> {
      this.runShooterEnum(mode);
    }, this).withName("Shooter.runShooterEnum" + mode.toString());
  }

  public Command stop() {
    return new RunCommand(() -> {
      flywheelIO.setRPM(0);
    }, this).withName("Intake.Stop");

  } 
}
