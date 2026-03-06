package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterAnglerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import lombok.Getter;

public class Shooter extends SubsystemBase {
  private final AnglerIO anglerIO;
  private final FlywheelIO flywheelIO;

  private final AnglerIOInputsAutoLogged anglerInputs = new AnglerIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  
  private ShooterModes shooterMode;

  @Getter private boolean zeroed = false;

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
    flywheelIO.setRPM(mode.speed);
    anglerIO.setPosition(mode.angle.magnitude());
  }

  public Command runAngler(ShooterModes mode) {
    return new RunCommand(() -> {
      // this.runShooterEnum(mode);
      anglerIO.setPosition(mode.angle.magnitude());
    }, this).withName("Shooter.runAngler" + mode.toString());
  } 

  public Command runShooterCommand(ShooterModes mode) {
    return new RunCommand(() -> {
      this.runShooterEnum(mode);
    }, this).withName("Shooter.runShooterEnum" + mode.toString());
  }

  public Command stop() {
    return new RunCommand(() -> {
      flywheelIO.setRPM(AngularVelocity.ofBaseUnits(0, RPM));
      anglerIO.setVoltage(0);
    }, this).withName("Intake.Stop");

  } 

  public InstantCommand setZero() {
    return new InstantCommand(() -> anglerIO.setZero(), this);
  }
  
  public Command homing() {
    return new SequentialCommandGroup(
      new RunCommand(() -> {
        zeroed = false;
        anglerIO.setVoltage(-2);
      }, this).until(
        () -> anglerInputs.currentAmps > ShooterAnglerConstants.homingCurrent.baseUnitMagnitude()
      ).withTimeout(2),
      new ParallelCommandGroup(
        new RunCommand(() -> {
          zeroed = true;
          anglerIO.setVoltage(0);
        }), null),
        setZero()
      );
  }
}