package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterAnglerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import lombok.Getter;

public class Shooter extends SubsystemBase {
  /*
   * 
   */
  private final AnglerIO anglerIO;
  private final FlywheelIO flywheelIO;

  private final AnglerIOInputsAutoLogged anglerInputs = new AnglerIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  
  private ShooterModes shooterMode;
  private static InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

  @Getter private boolean zeroed = false;

  public Shooter(AnglerIO anglerIO, FlywheelIO flywheelIO) {
    this.anglerIO = anglerIO;
    this.flywheelIO = flywheelIO;
    this.shooterMode = ShooterModes.IDLE;
    speedMap.put(10.1, 2500.0);
    speedMap.put(10.8, 3000.0);
  }
  
  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerInputs);
    Logger.processInputs("Shooter/Angler", anglerInputs);
    
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    Logger.recordOutput("Shooter/Mode", shooterMode);
    Logger.recordOutput("Shooter/Zeroed", zeroed);
    // System.out.println("***********************");
    // System.out.println(FlywheelIOTalonFX.plotOutput);
    // System.out.println("***********************");
  }

  public void runShooterEnum(ShooterModes mode) {
    this.shooterMode = mode;
    flywheelIO.setRPM(mode.speed);
    anglerIO.setPosition(mode.angle.magnitude());
  }

  public void runShooterCustom(double rpm, double position) {
    flywheelIO.setRPM(Units.RPM.of(rpm));
    anglerIO.setPosition(position);
  }

  // public Command runShootAuto(double dist) {
  //   return new RunCommand(() -> flywheelIO.setRPM(Units.RPM.of(speedMap.get(dist))), this);
  // }

  public Command runShootAuto(double dist) {
    return new RunCommand(() -> runShootAutotest(dist), this);
  }

   public void runShootAutotest(double dist) {
    System.out.println("((((((((((()))))))))))");
    System.out.println(dist);
    System.out.println(speedMap.get(dist));
    System.out.println(speedMap.get(10.1));
    System.out.println(speedMap.get(10.8));
    System.out.println("((((((((((()))))))))))");
    flywheelIO.setRPM(Units.RPM.of(speedMap.get(dist)));
  }

  public BooleanSupplier atSetpoint() {
    return () -> MathUtil.isNear(flywheelInputs.setpointRPM, flywheelInputs.leaderRPM, flywheelInputs.setpointRPM * 0.10);
  }

  public Command runAngler(ShooterModes mode) {
    return new RunCommand(() -> {
      // this.runShooterEnum(mode);
      anglerIO.setPosition(mode.angle.magnitude());
    }, this).withName("Shooter.runAngler" + mode.toString());
  } 

  public Command  runShooterCommand(ShooterModes mode) {
    return new RunCommand(() -> {
      this.runShooterEnum(mode);
    }, this).withName("Shooter.runShooterEnum" + mode.toString());
  }

  public Command runShooterVoltageCommand(double voltage){
    return new RunCommand(()-> {
      flywheelIO.setVoltage(voltage);
    });
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

  public Command tune() {
    return new RunCommand(() -> {
      flywheelIO.setRPM(AngularVelocity.ofRelativeUnits(ShooterFlywheelConstants.tuneDesiredSpeed.get(), RPM));
      anglerIO.setPosition(ShooterAnglerConstants.tuneDesiredPosition.get());
    }, this);
  }
  
  public Command homing() {
    return new SequentialCommandGroup(
      new RunCommand(() -> {
        zeroed = false;
        anglerIO.setVoltage(-1);
      }, this).until(
        () -> anglerInputs.currentAmps > ShooterAnglerConstants.homingCurrent.baseUnitMagnitude()
      ).withTimeout(2),
      Commands.runOnce(() -> {
        zeroed = true;
        anglerIO.setZero();
      })
      );
  }

  public double getPosition() {
    return anglerInputs.positionAngle;
  }

  public double getSpeed() {
    return flywheelInputs.leaderRPM;
  }

}