package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import org.ejml.dense.block.VectorOps_DDRB;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX follower  = new TalonFX(ShooterFlywheelConstants.followerId, Constants.CANIVORE_CANBUS);
  private final TalonFX leader = new TalonFX(ShooterFlywheelConstants.leaderId, Constants.CANIVORE_CANBUS);

  public static class FlywheelTalonFXConfig extends CTREMechanism.Config {
    public FlywheelTalonFXConfig() {
      super("FlywheelTalonFX", Constants.CANIVORE_CANBUS);
      configNeutralBrakeMode(ShooterFlywheelConstants.breakType);
      configFeedbackSensorSource(ShooterFlywheelConstants.feedbackSensorCTRE);
      configNeutralBrakeMode(ShooterFlywheelConstants.breakType);
      configPIDGains(ShooterFlywheelConstants.p, ShooterFlywheelConstants.i, ShooterFlywheelConstants.d);
      configGearRatio(ShooterFlywheelConstants.gearRatio);
      configMotorInverted(ShooterFlywheelConstants.inverted);
      configFeedForwardGains(0.35, 0.12, 0, 0);
    }
  }

  private StatusSignal<Voltage> leaderAppliedVoltage;
  private StatusSignal<AngularVelocity> leaderFlywheelRPM;
  private StatusSignal<Current> leaderAmps;

  private StatusSignal<Voltage> followerAppliedVoltage;
  private StatusSignal<AngularVelocity> followerFlywheelRPM;
  private StatusSignal<Current> followerAmps;
  public static VelocityVoltage plotOutput;
  public static double plotrps;
  // No clue stole from ModuleIO
  private final Debouncer flywheelConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

      private FlywheelTalonFXConfig config = new FlywheelTalonFXConfig();

  public FlywheelIOTalonFX() {
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderFlywheelRPM = leader.getVelocity();
    leaderAmps = leader.getStatorCurrent();
    
    followerAppliedVoltage = follower.getMotorVoltage();
    followerFlywheelRPM = follower.getVelocity();
    followerAmps = follower.getStatorCurrent();

    // config.talonConfig.Slot0.k;
    config.applyTalonConfig(leader);
    config.applyTalonConfig(follower);
    
    // will need to config whether aligned or inverted later
    follower.setControl(new Follower(ShooterFlywheelConstants.leaderId, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(50, leaderAppliedVoltage, leaderAmps, leaderFlywheelRPM, followerAppliedVoltage, followerAmps, followerFlywheelRPM);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    var flywheelStatus =  BaseStatusSignal.refreshAll(leaderAppliedVoltage, leaderAmps, leaderFlywheelRPM, followerAppliedVoltage, followerAmps, followerFlywheelRPM);

    inputs.flywheelConnected = flywheelConnectedDebounce.calculate(flywheelStatus.isOK());

    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderRPM = leaderFlywheelRPM.getValue().in(RPM);
    inputs.leaderAmps = leaderAmps.getValueAsDouble();

    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerRPM = followerFlywheelRPM.getValue().in(RPM);
    inputs.followerAmps = followerAmps.getValueAsDouble();

    if (plotrps > 0 && plotOutput.Velocity > 0) {
          SmartDashboard.putNumber("FlyhweelRPS", plotrps);
    SmartDashboard.putNumber("FlyhweelOutputVelocity", plotOutput.Velocity);
    }
    
  }

  @Override
  public void setRPM(double rpm) {
    AngularVelocity rps = Units.RotationsPerSecond.of(rpm / 60);
    AngularVelocity rps2 = RotationsPerSecond.of(4800 / 60);
    // VelocityVoltage output = config.velocityControl.withVelocity(rps);
    VelocityVoltage velocityOuput = new VelocityVoltage(rps2).withSlot(0);
    leader.setControl(velocityOuput);

    plotOutput = velocityOuput;
    // plotrps = rps2;

    

    System.out.println("******************");
    System.out.println(rps2);
    System.out.println("******************");
    System.out.println(velocityOuput);
    System.out.println("******************");
    // FIX RPM WHY NO WORK? rn its hardcoded voltage
    // if (rpm == 0 || rpm < 0) {
    //   leader.setVoltage(0);
    // }
    // else {
    //    leader.setVoltage(5.5);
    // }
  }
}
