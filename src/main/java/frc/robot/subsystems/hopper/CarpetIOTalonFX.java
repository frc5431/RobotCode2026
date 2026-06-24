package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetRollerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;
// import frc.robot.subsystems.intake.IntakeConstants.IntakeRollerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class CarpetIOTalonFX implements CarpetIO {
  private final TalonFX talon = new TalonFX(CarpetRollerConstants.id, Constants.CANIVORE_CANBUS);
  
  public final PIDController pid = new PIDController(CarpetRollerConstants.p.get(),
      CarpetRollerConstants.i.get(), CarpetRollerConstants.d.get());

  public static class CarpetIOTalonFXConfig extends CTREMechanism.Config {
    public CarpetIOTalonFXConfig() {
      super("RollerTalonFX",Constants.CANIVORE_CANBUS);
      // configPIDGains(CarpetRollerConstants.p, CarpetRollerConstants.i, CarpetRollerConstants.d);
      configNeutralBrakeMode(CarpetRollerConstants.breakType);
      configFeedbackSensorSource(CarpetRollerConstants.feedbackSensorCTRE);
      configSupplyCurrentLimit(CarpetRollerConstants.supplyLimit);
    }
  }
//pebus
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<AngularVelocity> rollerRPM;
  private StatusSignal<Current> currentAmps;
      public double setpointRPM = 0.0;


  // No clue what this means copied from ModuleIO
  private final Debouncer carpetConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private CarpetIOTalonFXConfig config = new CarpetIOTalonFXConfig();

  public CarpetIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    rollerRPM = talon.getVelocity();
    currentAmps = talon.getSupplyCurrent();
    config.applyTalonConfig(talon);
    
    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVoltage, currentAmps, rollerRPM);
  }

  @Override
  public void updateInputs(CarpetIOInputs inputs) {
    var carpetStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, rollerRPM);

    pid.setP(CarpetRollerConstants.p.get());
    pid.setI(CarpetRollerConstants.i.get());
    pid.setD(CarpetRollerConstants.d.get());

    inputs.rollerConnected = carpetConnectedDebounce.calculate(carpetStatus.isOK());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.RPM = rollerRPM.getValue().in(RPM);
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.setpointRPM = setpointRPM;
  }

  @Override
  public void setRollerVoltage(double voltage) {
    if(voltage > 0){
      talon.setVoltage(CarpetRollerConstants.testVoltage.get());
    }
    else {
      talon.setVoltage(0);
    }
    
  }

  @Override
  public void setRPM(AngularVelocity rpm) {
    setpointRPM = rpm.in(Units.RPM);

    Logger.recordOutput("/Carpet/Voltage", talon.getMotorVoltage().getValueAsDouble());
    AngularVelocity currentRPM = talon.getVelocity().getValue();
    double pidOutput = pid.calculate(currentRPM.in(Units.RPM), rpm.in(Units.RPM));

    double voltage = pidOutput + CarpetRollerConstants.kS.get()
        + CarpetRollerConstants.kV.get() * rpm.in(Units.RPM);

    voltage = Math.max(Math.min(voltage, 12), -12);

    talon.setVoltage(voltage);

    if (rpm.in(Units.RotationsPerSecond) > 0) {
      talon.setVoltage(voltage);
    } else {
      talon.set(0);
    }
  }
}
