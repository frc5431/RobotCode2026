package frc.robot.subsystems.shooter.angler;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterAnglerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class AnglerIOTalonFX implements AnglerIO {
  private final TalonFX talon = new TalonFX(ShooterAnglerConstants.id, Constants.CANBUS);

  public static class PivotTalonFXConfig extends CTREMechanism.Config {
    public PivotTalonFXConfig() {
      super("AnglerTalonFX", Constants.CANBUS);
      configPIDGains(ShooterAnglerConstants.p, ShooterAnglerConstants.i, ShooterAnglerConstants.d);
      configNeutralBrakeMode(ShooterAnglerConstants.breakType);
      configFeedbackSensorSource(ShooterAnglerConstants.feedbackSensorCTRE);
      configGearRatio(ShooterAnglerConstants.gearRatio);
      configSupplyCurrentLimit(ShooterAnglerConstants.supplyLimit);
    }
  }

  
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Angle> pivotPosition;
  private StatusSignal<Current> currentAmps;

  private PivotTalonFXConfig config = new PivotTalonFXConfig();

  public AnglerIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    pivotPosition = talon.getPosition();
    currentAmps = talon.getStatorCurrent();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, pivotPosition);

  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, pivotPosition);

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.positionAngle = pivotPosition.getValue().in(Rotation);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionAngle) {
    PositionVoltage mm = config.positionVoltage.withPosition(positionAngle);
      talon.setControl(mm);
  }
}