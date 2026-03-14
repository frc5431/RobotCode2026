package frc.robot.subsystems.shooter.angler;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterAnglerConstants;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class AnglerIOTalonFX implements AnglerIO {
  private final TalonFX talon = new TalonFX(ShooterAnglerConstants.id, Constants.CANIVORE_CANBUS);

  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 8.0));
  private TrapezoidProfile.State anglerSetpoint = new TrapezoidProfile.State();
  private PositionTorqueCurrentFOC anglerRequest = new PositionTorqueCurrentFOC(0);

  // public final ProfiledPIDController pid = new ProfiledPIDController(ShooterAnglerConstants.anglerP.get(),
  //     ShooterAnglerConstants.anglerI.get(), ShooterAnglerConstants.anglerD.get(), new TrapezoidProfile.Constraints(3.0, 8.0));

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
  private StatusSignal<AngularVelocity> velocityRPM;
  private double angleSetpoint;

  // No clue stole from ModuleIO
  private final Debouncer anglerConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private PivotTalonFXConfig config = new PivotTalonFXConfig();

  public AnglerIOTalonFX() {
    appliedVoltage = talon.getMotorVoltage();
    pivotPosition = talon.getPosition();
    currentAmps = talon.getSupplyCurrent();
    velocityRPM = talon.getVelocity();
    config.applyTalonConfig(talon);

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, currentAmps, pivotPosition, velocityRPM);

  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {

    // if (ShooterAnglerConstants.tunePID) {
    //   pid.setP(ShooterAnglerConstants.anglerP.get());
    //   pid.setI(ShooterAnglerConstants.anglerI.get());
    //   pid.setD(ShooterAnglerConstants.anglerD.get());
    // }

    var anglerStatus = BaseStatusSignal.refreshAll(appliedVoltage, currentAmps, pivotPosition);

    inputs.anglerConnected = anglerConnectedDebounce.calculate(anglerStatus.isOK());

    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.positionAngle = pivotPosition.getValue().in(Rotation);
    inputs.currentAmps = currentAmps.getValueAsDouble();

    Logger.recordOutput("/Shooter/Angler/DesiredAngle", angleSetpoint);
  }

  @Override
  public void setPosition(double angleSetpoint) {
    this.angleSetpoint = angleSetpoint;
    TrapezoidProfile.State currentState = new TrapezoidProfile.State(pivotPosition.getValue()
        .in(Rotation), velocityRPM.getValueAsDouble() / 60.0);
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(angleSetpoint, 0);
    anglerSetpoint = profile.calculate(0.02, currentState, goalState);

    anglerRequest.Position = anglerSetpoint.position;
    anglerRequest.Velocity = anglerSetpoint.velocity;
    talon.setControl(anglerRequest);
    
    // // PositionVoltage mm = config.positionVoltage.withPosition(positionAngle);
    // // talon.setControl(mm);
    // this.angleSetpoint = angleSetpoint;
    // // far is 0.75, near is 1
    // // angleSetpoint = ShooterAnglerConstants.bangBangAngle.getAsDouble();

    
    // double currentAngle = talon.getPosition().getValueAsDouble();
    // double voltage = 0;

    // // ShooterAnglerConstants.bangBangController.setTolerance(ShooterAnglerConstants.tolerance);
    // // ShooterAnglerConstants.bangBangController.setTolerance(ShooterAnglerConstants.tunableTolerance.getAsDouble());

    // voltage = pid.calculate(currentAngle, angleSetpoint);

    // voltage = Math.max(Math.min(voltage, 12), -12);

    // // talon.setVoltage( ShooterAnglerConstants.anglerkV.get());

    // talon.setVoltage(voltage);
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