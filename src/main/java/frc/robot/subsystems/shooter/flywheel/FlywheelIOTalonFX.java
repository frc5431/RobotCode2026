package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX followerFX  = new TalonFX(ShooterFlywheelConstants.followerId, Constants.CANBUS);
  private final TalonFX LeaderFX = new TalonFX(ShooterFlywheelConstants.leaderId, Constants.CANBUS);

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

  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Current> currentAmps;

  private FlywheelTalonFXConfig config = new FlywheelTalonFXConfig();

  public FlywheelIOTalonFX(TalonFX talon) {
    appliedVoltage = talon.getMotorVoltage();
    rollerRPM = talon.getVelocity();
    currentAmps = talon.getStatorCurrent();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, rollerRPM);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, rollerRPM);

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.RPM = rollerRPM.getValue().in(RPM);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setRPM(double rpm) {
    VelocityVoltage output = config.velocityControl.withVelocity(Units.RPM.of(rpm));
    LeaderFX.setControl(output);
  }
}
