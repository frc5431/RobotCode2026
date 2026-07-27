package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFeederConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.shooter.feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import lombok.Getter;

public class Shooter extends SubsystemBase {
  
  private final FeederIO feederIO;
  private final FlywheelIO flywheelIO;

  private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  
  private ShooterModes shooterMode;

  @Getter private boolean zeroed = false;

  public Shooter(FeederIO feederIO, FlywheelIO flywheelIO) {
    this.feederIO = feederIO;
    this.flywheelIO = flywheelIO;
    this.shooterMode = ShooterModes.IDLE;

    // Force-load so PID tunables publish to NT in sim/replay (sim IOs don't touch these).
    ShooterFlywheelConstants.testp.get();
    ShooterFeederConstants.p.get();

    
    }
  
  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("Shooter/Feeder", feederInputs);
    
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    Logger.recordOutput("Shooter/Mode", shooterMode);
    Logger.recordOutput("Shooter/Zeroed", zeroed);
    Logger.recordOutput("Shooter/ShootMapCalc", 0.0);

    
  }

  public void runShooterEnum(ShooterModes mode) {
    this.shooterMode = mode;
    flywheelIO.setRPM(mode.flywheelSpeed);
    feederIO.setVoltage(mode.feederVoltage.magnitude());
  }

  public Command runShooterCustom(DoubleSupplier flywheelRPM, DoubleSupplier feederRPM) {

    return new RunCommand(() -> {
      flywheelIO.setRPM(Units.RPM.of(flywheelRPM.getAsDouble()));
      feederIO.setRPM(Units.RPM.of(feederRPM.getAsDouble()));
    }, this);

  }

  public Command runShootAuto(DoubleSupplier dist) {
    return new RunCommand(() -> flywheelIO.setRPM(Units.RPM.of(ShooterMath.calculateSpeed(dist.getAsDouble()))), this);
  }


  public Command runShootMap(DoubleSupplier dist, DoubleSupplier feederRPM) {
    return new RunCommand(() -> {
      flywheelIO.setRPM(Units.RPM.of(ShooterMath.calculateSpeed(dist.getAsDouble())));
      feederIO.setRPM(Units.RPM.of(feederRPM.getAsDouble()));
    }, this);
  }

 
  public double getMapSpeed(double dist) {
    return ShooterMath.calculateSpeed(dist);
  }



  public BooleanSupplier atSetpoint() {
    return () -> MathUtil.isNear(flywheelInputs.setpointRPM, flywheelInputs.leftTopLeaderRPM, flywheelInputs.setpointRPM * 0.10);
  }

  public Command runFeeder(ShooterModes mode) {
    return new RunCommand(() -> {
      // this.runShooterEnum(mode);
      feederIO.setVoltage(mode.feederVoltage.magnitude());
    }, this).withName("Shooter.runFeeder" + mode.toString());
  } 

  public Command  runShooterCommand(ShooterModes mode) {
    return new RunCommand(() -> {
      this.runShooterEnum(mode);
    }, this).withName("Shooter.runShooterEnum" + mode.toString());
  }

  public Command runCustomVoltageCommand(double feederVoltage, double flywheelVoltage){
    return new RunCommand(()-> {
      flywheelIO.setVoltage(feederVoltage);
      feederIO.setVoltage(flywheelVoltage);
    });
  }


  public Command stop() {
    return new RunCommand(() -> {
      flywheelIO.setRPM(AngularVelocity.ofBaseUnits(0, RPM));
      feederIO.setVoltage(0);
    }, this).withName("Intake.Stop");

  } 






  

    public Command feederRPM(AngularVelocity RPM){
    return new RunCommand(() -> {
      feederIO.setRPM(RPM);
    }, this);

  }


  public double getFeederVoltage() {
    return feederInputs.leaderApliedVoltage;
  }

  public double getFlywheelSpeed() {
    return flywheelInputs.leftTopLeaderRPM;
  }

}