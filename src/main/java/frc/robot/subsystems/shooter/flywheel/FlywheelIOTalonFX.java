package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX follower = new TalonFX(ShooterFlywheelConstants.followerId, Constants.CANIVORE_CANBUS);
  private final TalonFX leader = new TalonFX(ShooterFlywheelConstants.leaderId, Constants.CANIVORE_CANBUS);

  public final PIDController pid = new PIDController(ShooterFlywheelConstants.testp.get(), ShooterFlywheelConstants.testi.get(), ShooterFlywheelConstants.testd.get());

  public static class FlywheelTalonFXConfig extends CTREMechanism.Config {
    public FlywheelTalonFXConfig() {
      super("FlywheelTalonFX", Constants.CANIVORE_CANBUS);
      configNeutralBrakeMode(ShooterFlywheelConstants.breakType);
      configFeedbackSensorSource(ShooterFlywheelConstants.feedbackSensorCTRE);
      // configPIDGains(0, ShooterFlywheelConstants.p, ShooterFlywheelConstants.i, ShooterFlywheelConstants.d);
      configGearRatio(ShooterFlywheelConstants.gearRatio);
      configMotorInverted(ShooterFlywheelConstants.inverted);
      configFeedForwardGains(0, 0.35, 0.12, 0, 0);
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
  private final Debouncer flywheelConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private FlywheelTalonFXConfig config = new FlywheelTalonFXConfig();

  public FlywheelIOTalonFX() {
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderFlywheelRPM = leader.getVelocity();
    leaderAmps = leader.getStatorCurrent();

    followerAppliedVoltage = follower.getMotorVoltage();
    followerFlywheelRPM = follower.getVelocity();
    followerAmps = follower.getStatorCurrent();

    // TalonFXConfiguration config1 = new TalonFXConfiguration();
    // config1.
    // config.talonConfig.Slot0.k;
    config.applyTalonConfig(leader);
    config.applyTalonConfig(follower);

    // will need to config whether aligned or inverted later
    follower.setControl(new Follower(ShooterFlywheelConstants.leaderId, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(50, leaderAppliedVoltage, leaderAmps, leaderFlywheelRPM,
        followerAppliedVoltage, followerAmps, followerFlywheelRPM);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    pid.setP(ShooterFlywheelConstants.testp.get());
    pid.setI(ShooterFlywheelConstants.testi.get());
    pid.setD(ShooterFlywheelConstants.testd.get());
 

    var flywheelStatus = BaseStatusSignal.refreshAll(leaderAppliedVoltage, leaderAmps, leaderFlywheelRPM,
        followerAppliedVoltage, followerAmps, followerFlywheelRPM);

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

    SmartDashboard.putNumber("Flywheel RPM", leader.getVelocity().getValue().in(Units.RPM));

  }
  

  @Override
  public void setRPM(AngularVelocity rpm){
    Logger.recordOutput("/Shooter/DesiredRPM", rpm);
    Logger.recordOutput("/Shooter/Voltage", leader.getMotorVoltage().getValueAsDouble());
    AngularVelocity currentRPM = leader.getVelocity().getValue();
    double pidOutput = pid.calculate(currentRPM.in(Units.RPM), rpm.in(Units.RPM));

    double voltage = pidOutput + ShooterFlywheelConstants.testkS.get() + ShooterFlywheelConstants.testkV.get() * rpm.in(Units.RPM);

    voltage = Math.max(Math.min(voltage, 12), -12);

    leader.setVoltage(voltage);

    if(rpm.in(Units.RotationsPerSecond) > 0){
    leader.setVoltage(voltage);
    }
    else {
      leader.set(0);
    }

    // System.out.println("******************");
    // System.out.println(ShooterFlywheelConstants.testp.getAsDouble());
    // System.out.println("******************");
    // if (rpm == 0 || rpm < 0) {
    //   leader.setVoltage(0);
    // } else {
    //   leader.setVoltage(5);
    // }
  }

  @Override
  public void setVoltage(double voltage) {

     leader.setVoltage(voltage);
  }
}
