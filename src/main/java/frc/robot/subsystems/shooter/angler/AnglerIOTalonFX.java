package frc.robot.subsystems.shooter.angler;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterAnglerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class AnglerIOTalonFX implements AnglerIO {
  private final TalonFX talon = new TalonFX(ShooterAnglerConstants.id, Constants.CANIVORE_CANBUS);

  public final PIDController pid = new PIDController(ShooterAnglerConstants.anglerP.get(), ShooterAnglerConstants.anglerI.get(), ShooterAnglerConstants.anglerD.get());

  public static class PivotTalonFXConfig extends CTREMechanism.Config {
    public PivotTalonFXConfig() {
      super("AnglerTalonFX", Constants.CANIVORE_CANBUS);
      configPIDGains(ShooterAnglerConstants.p, ShooterAnglerConstants.i, ShooterAnglerConstants.d);
      configNeutralBrakeMode(ShooterAnglerConstants.breakType);
      configFeedbackSensorSource(ShooterAnglerConstants.feedbackSensorCTRE);
      configGearRatio(ShooterAnglerConstants.gearRatio);
      configSupplyCurrentLimit(ShooterAnglerConstants.supplyLimit);
      configForwardSoftLimit(voltageCompSaturation, false);
    }
  }

  
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Angle> pivotPosition;
  private StatusSignal<Current> currentAmps;

  // No clue stole from ModuleIO
  private final Debouncer anglerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

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

    if (ShooterAnglerConstants.tunePID) {
      pid.setP(ShooterAnglerConstants.anglerP.get());
      pid.setI(ShooterAnglerConstants.anglerI.get());
      pid.setD(ShooterAnglerConstants.anglerD.get());
    }

    var anglerStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, pivotPosition);

    inputs.anglerConnected = anglerConnectedDebounce.calculate(anglerStatus.isOK());

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.positionAngle = pivotPosition.getValue().in(Rotation);
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionAngle) {
    // PositionVoltage mm = config.positionVoltage.withPosition(positionAngle);
    //   talon.setControl(mm);

    Logger.recordOutput("/ShooterAngler/DesiredAngle", positionAngle);
    Logger.recordOutput("/ShooterAngler/Voltage", talon.getMotorVoltage().getValueAsDouble());
    Angle currentAngle = talon.getPosition().getValue();
    double angleDifference = positionAngle - currentAngle.in(Radians);
    double voltage = 0;
    if (angleDifference > 0.05) {
        voltage = ShooterAnglerConstants.anglerP.get();
    } else if (angleDifference < -0.05) {
      voltage = -ShooterAnglerConstants.anglerD.get();
    } else {
      voltage = ShooterAnglerConstants.anglerkS.get();
    }
    // if (currentAngle.in(Rotations) > positionAngle) {

    // }
    // double pidOutput = pid.calculate(currentAngle.in(Units.Rotations), positionAngle);

    // double voltage = pidOutput + ShooterAnglerConstants.anglerkS.get() + ShooterAnglerConstants.anglerkV.get() * positionAngle;

    voltage = Math.max(Math.min(voltage, 12), -12);

    // System.out.println(voltage);
    // talon.setVoltage( ShooterAnglerConstants.anglerkV.get());

    talon.setVoltage(voltage);

    // if(positionAngle > 0){
    // talon.setVoltage(voltage);
    // }
    // else {
    //   talon.set(0);
    // }

  }

  @Override
  public void setVoltage(double voltage) {
    talon.setVoltage(voltage);
  }

  @Override
  public void setZero() {
    talon.setControl(new NeutralOut());
    talon.setPosition(0.0);
    talon.getPosition().waitForUpdate(0.1);   
  }
}