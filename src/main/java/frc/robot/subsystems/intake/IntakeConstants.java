package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public  class IntakeConstants {
  
  public enum IntakeMode {

    STOW(Units.Volts.of(0.0), Units.Degrees.of(0.0)),
    OUT_IDLE(Units.Volts.of(0.0), Units.Degrees.of(180.0)),
    INTAKE(Units.Volts.of(-3), Units.Degrees.of(0.0)),
    OUTTAKE(Units.Volts.of(2.8), Units.Degrees.of(180.0));

    public Voltage voltage;
    public Angle position;

    IntakeMode(Voltage voltage, Angle position) {
      this.voltage = voltage;
      this.position = position;
    }
  }

  public static final class IntakeRollerConstants {

    public static final boolean attached = true;

    public static final int leaderId = 61;
    public static final int followerId = 62;

    PIDConstants pidConstants = new PIDConstants(1, 0, 0);
    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    public static final double maxIAccum = 0.2;

    public static final double gearRatio = 1 / 1;

    public static final boolean invert = false;
    public static final boolean gravityType = false;
    public static final boolean breakType = false;

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.RotorSensor;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kPrimaryEncoder;

    public static final boolean useFMaxRotation = true;
    public static final boolean useRMaxRotation = true;
    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(5);

    public static final Current stallLimit = Units.Amps.of(70);
    public static final Current supplyLimit = Units.Amps.of(50);
  }

  public static final class IntakePivotConstants {
    public static final boolean attached = true;

    public static final int id = 14;

    public static double p =.003;
    public static final double i = 0;
    public static final double d = 0.00002;
    public static final double s = 0.375;
    public static final double v = 0.0021;
    public static final double maxIAccum = 0.2;

    public static final double gearRatio = 1 / 1;

    public static final boolean invert = false;
    public static final boolean gravityType = false;
    public static final boolean breakType = true;

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kAbsoluteEncoder;

    public static final boolean useFMaxRotation = true;
    public static final boolean useRMaxRotation = true;
    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(5);

    public static final Current stallLimit = Units.Amps.of(80);
    public static final Current supplyLimit = Units.Amps.of(60);

  }
}