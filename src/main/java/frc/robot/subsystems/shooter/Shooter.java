package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  /*
   * 
   */
  private final FeederIO feederIO;
  private final FlywheelIO flywheelIO;

  private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  
  private ShooterModes shooterMode;
  private static InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

  @Getter private boolean zeroed = false;

  public Shooter(FeederIO feederIO, FlywheelIO flywheelIO) {
    this.feederIO = feederIO;
    this.flywheelIO = flywheelIO;
    this.shooterMode = ShooterModes.IDLE;
    speedMap.put(1.9, 2000.0);
    speedMap.put(2.75, 2500.0);
    speedMap.put(2.9, 2600.0);
    speedMap.put(3.1, 2700.0);
    speedMap.put(3.3, 2800.0);
    speedMap.put(3.5, 3000.0);
    speedMap.put(3.8, 3500.0);
  }
  
  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("Shooter/Feeder", feederInputs);
    
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
    flywheelIO.setRPM(mode.flywheelSpeed);
    feederIO.setVoltage(mode.feederVoltage.magnitude());
  }

  public void runShooterCustom(double rpm, double feederVoltage) {
    flywheelIO.setRPM(Units.RPM.of(rpm));
    feederIO.setVoltage(feederVoltage);
  }

  public Command runShootAuto(DoubleSupplier dist) {
    return new RunCommand(() -> flywheelIO.setRPM(Units.RPM.of(speedMap.get(dist.getAsDouble()))), this);
  }

  // public Command runShootAuto(DoubleSupplier dist) {
  //   return new RunCommand(() -> runShootAutotest(dist.getAsDouble()), this);
  // }

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

  // public InstantCommand setZero() {
  //   return new InstantCommand(() -> feederIO.setZero(), this);
  // }

  public Command tune() {
    return new RunCommand(() -> {
      flywheelIO.setRPM(AngularVelocity.ofRelativeUnits(ShooterFlywheelConstants.tuneDesiredSpeed.get(), RPM));
      feederIO.setRPM(AngularVelocity.ofRelativeUnits(ShooterFeederConstants.tuneDesiredSpeed.get(), RPM));
    }, this);
  }
  
  // public Command homing() {
  //   return new SequentialCommandGroup(
  //     new RunCommand(() -> {
  //       zeroed = false;
  //       feederIO.setVoltage(-1);
  //     }, this).until(
  //       () -> feederInputs.leaderCurrentAmps > ShooterFeederConstants.homingCurrent.baseUnitMagnitude()
  //     ).withTimeout(2),
  //     Commands.runOnce(() -> {
  //       zeroed = true;
  //       feederIO.setZero();
  //     })
  //     );
  // }

  public double getFeederVoltage() {
    return feederInputs.leaderApliedVoltage;
  }

  public double getFlywheelSpeed() {
    return flywheelInputs.leftTopLeaderRPM;
  }

}