package frc.robot.subsystems.intake.pivot;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;

import frc.robot.Constants.IntakePivotConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class PivotIOSparkFlex implements PivotIO {
    private final SparkFlex sparkFlex = new SparkFlex(0, null);
    private final RelativeEncoder encoder = sparkFlex.getEncoder();

    public static class PivotSparkFlexConfig extends REVMechanism.Config {
        public PivotSparkFlexConfig() {
        super("PivotSparkFlex", IntakePivotConstants.id);
        configPIDGains(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
        configFeedbackSensorSource(IntakePivotConstants.feedbackSensorREV);
        // configGear(PivotConstants.gearRat);
        // configGravity(PivotConstants.gravityType);
        configSmartCurrentLimit(IntakePivotConstants.stallLimit, IntakePivotConstants.supplyLimit);
        configSmartStallCurrentLimit(IntakePivotConstants.stallLimit);
        configReverseSoftLimit(
            IntakePivotConstants.maxReverseRotation, IntakePivotConstants.useRMaxRotation);
        configForwardSoftLimit(
          IntakePivotConstants.maxFowardRotation, IntakePivotConstants.useFMaxRotation);
        }
    } 

    public PivotIOSparkFlex() {
        sparkFlex.configure(new PivotSparkFlexConfig().sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        ifOk(sparkFlex, encoder::getPosition, (value) -> inputs.position = value);
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
