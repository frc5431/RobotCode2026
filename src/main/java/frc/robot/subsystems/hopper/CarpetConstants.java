package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class CarpetConstants {
  public enum CarpetModes {
    IDLE(Units.Volts.of(0.0)),
    INTAKE(Units.Volts.of(5.4)),
    OUTTAKE(Units.Volts.of(-2.8));

    public Voltage voltage;
    
    CarpetModes(Voltage voltage) {
      this.voltage = voltage;
    }
  }
  
  public static final class CarpetRollerConstants {
    public static final boolean attached = true;

    public static final int id = -1;

    PIDConstants pidConstants = new PIDConstants(1, 0, 0);
    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    public static final double maxIAccum = 0.2;

    public static final double gearRatio = 1 / 1;

    public static final boolean invert = false;
    public static final boolean gravityType = false;
    public static final boolean breakType = false;

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kPrimaryEncoder;

    public static final boolean useFMaxRotation = true;
    public static final boolean useRMaxRotation = true;
    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(5);

    public static final Current stallLimit = Units.Amps.of(80);
    public static final Current supplyLimit = Units.Amps.of(60);
  }

}
