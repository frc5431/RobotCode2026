package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX leader = new TalonFX(IntakeRollerConstants.leaderId, Constants.RIO_CANBUS);
  private final TalonFX follower = new
  TalonFX(IntakeRollerConstants.followerId, Constants.RIO_CANBUS);

  public static class RollerTalonFXConfig extends CTREMechanism.Config {
    public RollerTalonFXConfig() {
      super("RollerTalonFX", Constants.RIO_CANBUS);
      configNeutralBrakeMode(IntakeRollerConstants.breakType);
      configFeedbackSensorSource(IntakeRollerConstants.feedbackSensorCTRE);
      // configSupplyCurrentLimit(IntakeRollerConstants.supplyLimit);
      
    }
  }

  private StatusSignal<Voltage> leaderAppliedVoltage;
  private StatusSignal<AngularVelocity> leaderRPM;
  private StatusSignal<Current> leaderCurrentAmps;

  public double setpointRPM = 0.0;

  public final PIDController pid = new PIDController(IntakeRollerConstants.p.get(),
      IntakeRollerConstants.i.get(), IntakeRollerConstants.d.get());

  private StatusSignal<Voltage> followerAppliedVoltage;
  private StatusSignal<AngularVelocity> followerRPM;
  private StatusSignal<Current> followerCurrentAmps;

  // No clue what this means copied from ModuleIO
  private final Debouncer rollerConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private RollerTalonFXConfig config = new RollerTalonFXConfig();

  public RollerIOTalonFX() {
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderRPM = leader.getVelocity();
    leaderCurrentAmps = leader.getSupplyCurrent();

    followerAppliedVoltage = follower.getMotorVoltage();
    followerRPM = follower.getVelocity();
    followerCurrentAmps = follower.getSupplyCurrent();

    config.applyTalonConfig(leader);
    config.applyTalonConfig(follower);

    // will need to config whether aligned or inverted later
    follower.setControl(new Follower(IntakeRollerConstants.leaderId,
    MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(100, leaderAppliedVoltage, leaderCurrentAmps, leaderRPM);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    var rollerStatus = BaseStatusSignal.refreshAll(leaderAppliedVoltage, leaderCurrentAmps, leaderRPM);

    pid.setP(IntakeRollerConstants.p.get());
    pid.setI(IntakeRollerConstants.i.get());
    pid.setD(IntakeRollerConstants.d.get());

    inputs.rollerConnected =
    rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderRPM = leaderRPM.getValue().in(RPM);
    inputs.leaderCurrentAmps = leaderCurrentAmps.getValueAsDouble();

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.setpointRPM = setpointRPM;
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerRPM = followerRPM.getValue().in(RPM);
    inputs.followerCurrentAmps = followerCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void setRPM(AngularVelocity rpm) {
    setpointRPM = rpm.in(Units.RPM);

    Logger.recordOutput("/Intake/Roller/Voltage", leader.getMotorVoltage().getValueAsDouble());
    AngularVelocity currentRPM = leader.getVelocity().getValue();
    double pidOutput = pid.calculate(currentRPM.in(Units.RPM), rpm.in(Units.RPM));

    double voltage = pidOutput + IntakeRollerConstants.kS.get()
        + IntakeRollerConstants.kV.get() * rpm.in(Units.RPM);

    voltage = Math.max(Math.min(voltage, 12), -12);

    leader.setVoltage(voltage);

    if (rpm.in(Units.RotationsPerSecond) > 0) {
      leader.setVoltage(voltage);
    } else {
      leader.set(0);
    }
  }

}
