package frc.robot.subsystems.intake.pivot;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterFlywheelConstants;

public class PivotIOSparkFlex implements PivotIO {
    private final SparkFlex sparkFlex = new SparkFlex(IntakePivotConstants.id, MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkFlex.getEncoder();
    public final PIDController pid = new PIDController(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
    private final SparkFlexConfig config = new SparkFlexConfig();


    public PivotIOSparkFlex() {
        // CONFIG
        config.closedLoop.feedbackSensor(ShooterFlywheelConstants.feedbackSensorREV);
        config.smartCurrentLimit(
            (int) IntakePivotConstants.stallLimit.in(Units.Amps), (int) IntakePivotConstants.supplyLimit.in(Units.Amps));
        config.closedLoop.pid(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
        config.softLimit.reverseSoftLimit(IntakePivotConstants.maxReverseRotation.in(Units.Rotations));
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit(IntakePivotConstants.maxReverseRotation.in(Units.Rotations));
        config.softLimit.forwardSoftLimitEnabled(true);

        sparkFlex.configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pid.setP(ShooterFlywheelConstants.testp.get());
        pid.setI(ShooterFlywheelConstants.testi.get());
        pid.setD(ShooterFlywheelConstants.testd.get());

        ifOk(sparkFlex, encoder::getPosition, (value) -> inputs.positionAngle = value);
        ifOk(sparkFlex, sparkFlex::getBusVoltage, (value) -> inputs.appliedVoltage = value);
        ifOk(sparkFlex, sparkFlex::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        sparkFlex.setVoltage(voltage);
    }

    @Override
    public void setPosition(double positionAngle) {
        sparkFlex.getClosedLoopController().setSetpoint((positionAngle), ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
    }
}
