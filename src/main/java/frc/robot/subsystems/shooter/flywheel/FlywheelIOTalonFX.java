package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX follower  = new TalonFX(ShooterFlywheelConstants.followerId, Constants.CANBUS);
  private final TalonFX leader = new TalonFX(ShooterFlywheelConstants.leaderId, Constants.CANBUS);

  public static class FlywheelTalonFXConfig extends CTREMechanism.Config {
    public FlywheelTalonFXConfig() {
      super("FlywheelTalonFX", Constants.CANBUS);
      configNeutralBrakeMode(ShooterFlywheelConstants.breakType);
      configFeedbackSensorSource(ShooterFlywheelConstants.feedbackSensorCTRE);
      configNeutralBrakeMode(ShooterFlywheelConstants.breakType);
      configPIDGains(ShooterFlywheelConstants.p, ShooterFlywheelConstants.i, ShooterFlywheelConstants.d);
      configGearRatio(ShooterFlywheelConstants.gearRatio);
      configMotorInverted(ShooterFlywheelConstants.inverted);
    }
  }

  private StatusSignal<Voltage> leaderAppliedVoltage;
  private StatusSignal<AngularVelocity> leaderFlywheelRPM;
  private StatusSignal<Current> leaderAmps;

  private StatusSignal<Voltage> followerAppliedVoltage;
  private StatusSignal<AngularVelocity> followerFlywheelRPM;
  private StatusSignal<Current> followerAmps;

  private FlywheelTalonFXConfig config = new FlywheelTalonFXConfig();

  public FlywheelIOTalonFX() {
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderFlywheelRPM = leader.getVelocity();
    leaderAmps = leader.getStatorCurrent();
    
    followerAppliedVoltage = follower.getMotorVoltage();
    followerFlywheelRPM = follower.getVelocity();
    followerAmps = follower.getStatorCurrent();

    config.applyTalonConfig(leader);
    config.applyTalonConfig(follower);
    
    // will need to config whether aligned or inverted later
    follower.setControl(new Follower(ShooterFlywheelConstants.leaderId, MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(50, leaderAppliedVoltage, leaderAmps, leaderFlywheelRPM, followerAppliedVoltage, followerAmps, followerFlywheelRPM);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(leaderAppliedVoltage, leaderAmps, leaderFlywheelRPM, followerAppliedVoltage, followerAmps, followerFlywheelRPM);

    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderRPM = leaderFlywheelRPM.getValue().in(RPM);
    inputs.leaderAmps = leaderAmps.getValueAsDouble();

    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerRPM = followerFlywheelRPM.getValue().in(RPM);
    inputs.followerAmps = followerAmps.getValueAsDouble();
  }

  @Override
  public void setRPM(double rpm) {
    VelocityVoltage output = config.velocityControl.withVelocity(Units.RPM.of(rpm));
    leader.setControl(output);
  }
}
