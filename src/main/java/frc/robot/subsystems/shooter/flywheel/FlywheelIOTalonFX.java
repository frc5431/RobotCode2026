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
  private final TalonFX leftBackLeader = new TalonFX(ShooterFlywheelConstants.leftBackId, Constants.RIO_CANBUS);

  private final TalonFX leftFrontFollower = new TalonFX(ShooterFlywheelConstants.leftFrontId, Constants.RIO_CANBUS);
 
  private final TalonFX rightFrontFollower = new TalonFX(ShooterFlywheelConstants.rightFrontId, Constants.RIO_CANBUS);

  private final TalonFX rightBottomFollower = new TalonFX(ShooterFlywheelConstants.rightBackId, Constants.RIO_CANBUS);

  public final PIDController pid = new PIDController(ShooterFlywheelConstants.testp.get(), ShooterFlywheelConstants.testi.get(), ShooterFlywheelConstants.testd.get());
  
  public static class FlywheelTalonFXConfig extends CTREMechanism.Config {
    public FlywheelTalonFXConfig() {
      super("FlywheelTalonFX", Constants.CANIVORE_CANBUS);
      configNeutralBrakeMode(ShooterFlywheelConstants.breakType);
      configFeedbackSensorSource(ShooterFlywheelConstants.feedbackSensorCTRE);
      
      configGearRatio(ShooterFlywheelConstants.gearRatio);
      configMotorInverted(ShooterFlywheelConstants.inverted);
      configFeedForwardGains(0, 0.35, 0.12, 0, 0);
      configSupplyCurrentLimit(ShooterFlywheelConstants.supplyLimit);
      configStatorCurrentLimit(Amps.of(100));
    }
  }

  private StatusSignal<Voltage> leftTopAppliedVoltage;
  private StatusSignal<AngularVelocity> leftTopFlywheelRPM;
  private StatusSignal<Current> leftTopAmps;

  private StatusSignal<Voltage> leftBottomAppliedVoltage;
  private StatusSignal<AngularVelocity> leftBottomFlywheelRPM;
  private StatusSignal<Current> leftBottomAmps;

  private StatusSignal<Voltage> rightTopAppliedVoltage;
  private StatusSignal<AngularVelocity> rightTopFlywheelRPM;
  private StatusSignal<Current> rightTopAmps;

  private StatusSignal<Voltage> rightBottomAppliedVoltage;
  private StatusSignal<AngularVelocity> rightBottomFlywheelRPM;
  private StatusSignal<Current> rightBottomAmps;

  public static VelocityVoltage plotOutput;
  public static double plotrps;
  

  public double setpointRPM = 0.0;
  // No clue stole from ModuleIO
  private final Debouncer flywheelConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private FlywheelTalonFXConfig leftConfig = new FlywheelTalonFXConfig();

  private FlywheelTalonFXConfig rightConfig = new FlywheelTalonFXConfig();

  public FlywheelIOTalonFX() {
    leftTopAppliedVoltage = leftBackLeader.getMotorVoltage();
    leftTopFlywheelRPM = leftBackLeader.getVelocity();
    leftTopAmps = leftBackLeader.getSupplyCurrent();

    leftBottomAppliedVoltage = leftFrontFollower.getMotorVoltage();
    leftBottomFlywheelRPM = leftFrontFollower.getVelocity();
    leftBottomAmps = leftFrontFollower.getSupplyCurrent();

    rightTopAppliedVoltage = rightFrontFollower.getMotorVoltage();
    rightTopFlywheelRPM = rightFrontFollower.getVelocity();
    rightTopAmps = rightFrontFollower.getSupplyCurrent();

    rightBottomAppliedVoltage = rightBottomFollower.getMotorVoltage();
    rightBottomFlywheelRPM = rightBottomFollower.getVelocity();
    rightBottomAmps = rightBottomFollower.getSupplyCurrent();

    // might need to do  
    // rightConfig.configMotorInverted(false);
    
    leftConfig.applyTalonConfig(leftBackLeader);
    leftConfig.applyTalonConfig(leftFrontFollower);

    rightConfig.applyTalonConfig(rightFrontFollower);
    rightConfig.applyTalonConfig(rightBottomFollower);

    // will need to config whether aligned or inverted later
    leftFrontFollower.setControl(new Follower(ShooterFlywheelConstants.leftBackId, MotorAlignmentValue.Aligned));

    rightBottomFollower.setControl(new Follower(ShooterFlywheelConstants.leftBackId, MotorAlignmentValue.Opposed));

    rightFrontFollower.setControl(new Follower(ShooterFlywheelConstants.leftBackId, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(100, 
        leftTopAppliedVoltage, leftTopAmps, leftTopFlywheelRPM,
        leftBottomAppliedVoltage, leftBottomAmps, leftBottomFlywheelRPM, 
        rightTopAppliedVoltage, rightTopAmps, rightTopFlywheelRPM,
        rightBottomAppliedVoltage, rightBottomAmps, rightBottomFlywheelRPM);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    pid.setP(ShooterFlywheelConstants.testp.get());
    pid.setI(ShooterFlywheelConstants.testi.get());
    pid.setD(ShooterFlywheelConstants.testd.get());
 

    var flywheelStatus = BaseStatusSignal.refreshAll(
        leftTopAppliedVoltage, leftTopAmps, leftTopFlywheelRPM,
        leftBottomAppliedVoltage, leftBottomAmps, leftBottomFlywheelRPM,
        rightTopAppliedVoltage, rightTopAmps, rightTopFlywheelRPM,
        rightBottomAppliedVoltage, rightBottomAmps, rightBottomFlywheelRPM);

    inputs.flywheelConnected = flywheelConnectedDebounce.calculate(flywheelStatus.isOK());

    inputs.leftTopLeaderAppliedVoltage = leftTopAppliedVoltage.getValueAsDouble();
    inputs.leftTopLeaderRPM = leftTopFlywheelRPM.getValue().in(RPM);
    inputs.leftTopLeaderAmps = leftTopAmps.getValueAsDouble();

    inputs.leftBottomFollowerAppliedVoltage = leftBottomAppliedVoltage.getValueAsDouble();
    inputs.leftBottomFollowerRPM = leftBottomFlywheelRPM.getValue().in(RPM);
    inputs.leftBottomFollowerAmps = leftBottomAmps.getValueAsDouble();

    inputs.rightTopFollowerAppliedVoltage = rightTopAppliedVoltage.getValueAsDouble();
    inputs.rightTopFollowerRPM = rightTopFlywheelRPM.getValue().in(RPM);
    inputs.rightTopFollowerAmps = rightTopAmps.getValueAsDouble();

    inputs.rightBottomFollowerAppliedVoltage = rightBottomAppliedVoltage.getValueAsDouble();
    inputs.rightBottomFollowerRPM = rightBottomFlywheelRPM.getValue().in(RPM);
    inputs.rightBottomFollowerAmps = rightBottomAmps.getValueAsDouble();

    inputs.setpointRPM = setpointRPM;
    if (plotrps > 0 && plotOutput.Velocity > 0) {
      SmartDashboard.putNumber("FlyhweelRPS", plotrps);
      SmartDashboard.putNumber("FlyhweelOutputVelocity", plotOutput.Velocity);
    }

    SmartDashboard.putNumber("Flywheel RPM", leftBackLeader.getVelocity().getValue().in(Units.RPM));

  }
  

  @Override
  public void setRPM(AngularVelocity rpm){
    setpointRPM = rpm.in(Units.RPM);
    
    Logger.recordOutput("/Shooter/Voltage", leftBackLeader.getMotorVoltage().getValueAsDouble());
    AngularVelocity currentRPM = leftBackLeader.getVelocity().getValue();
    double pidOutput = pid.calculate(currentRPM.in(Units.RPM), rpm.in(Units.RPM));

    double voltage = pidOutput + ShooterFlywheelConstants.testkS.get() + ShooterFlywheelConstants.testkV.get() * rpm.in(Units.RPM);

    voltage = Math.max(Math.min(voltage, 12), -12);

    leftBackLeader.setVoltage(voltage);

    if(rpm.in(Units.RotationsPerSecond) > 0){
    leftBackLeader.setVoltage(voltage);
    }
    else {
      leftBackLeader.set(0);
    }

    
   
  }

  @Override
  public void setVoltage(double voltage) {
     leftBackLeader.setVoltage(voltage);
  }
}
