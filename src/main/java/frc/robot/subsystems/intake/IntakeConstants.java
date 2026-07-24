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
    INTAKE(Units.Volts.of(6), Units.Degrees.of(0.0)),
    OUTTAKE(Units.Volts.of(-2.8), Units.Degrees.of(180.0)),
    INTAKE_MORE(Units.Volts.of(-9),  Units.Degrees.of(0.0));
    public Voltage voltage;
    public Angle position;

    IntakeMode(Voltage voltage, Angle position) {
      this.voltage = voltage;
      this.position = position;
    }
  }

  public static final class IntakeRollerConstants {

    public static final boolean attached = true;

    public static final int leaderId = 62;
    public static final int followerId = 61;

    PIDConstants pidConstants = new PIDConstants(1, 0, 0); //useless
    public static final double maxIAccum = 0.2;

    public static final double gearRatio = 1 / 1;

    public static final boolean invert = false;
    public static final boolean gravityType = false;
    public static final boolean breakType = false;

    public static LoggedNetworkNumber p = new LoggedNetworkNumber("/Tuning/Intake/Roller/P", 0);
    public static final LoggedNetworkNumber i = new LoggedNetworkNumber("/Tuning/Intake/Roller/I", 0);
    public static final LoggedNetworkNumber d = new LoggedNetworkNumber("/Tuning/Intake/Roller/D", 0.0);
    public static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Intake/Roller/kS", 0);
    public static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Intake/Roller/kV", 0);
    public static final LoggedNetworkNumber tuneDesiredSpeed = new LoggedNetworkNumber("/Tuning/Intake/Roller/desiredSpeed",0);

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.RotorSensor;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kPrimaryEncoder;

    public static final boolean useFMaxRotation = true;
    public static final boolean useRMaxRotation = true;
    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(5);

    public static final Current stallLimit = Units.Amps.of(80);
    public static final Current supplyLimit = Units.Amps.of(85);

    
  }

  public static final class IntakePivotConstants {
    public static final boolean attached = true;

    public static final int id = 14;
    public static final int cancoderId = 44;

    public static final double EncoderOffset = 0.0;
    public static final boolean EncoderInverted = true;
    
    // Live-tunable over NetworkTables (AdvantageScope: /Tuning/Intake/Pivot/...).
    public static final LoggedNetworkNumber p = new LoggedNetworkNumber("/Tuning/Intake/Pivot/P", 15);
    public static final LoggedNetworkNumber i = new LoggedNetworkNumber("/Tuning/Intake/Pivot/I", 0);
    public static final LoggedNetworkNumber d = new LoggedNetworkNumber("/Tuning/Intake/Pivot/D", 0.5);
    public static final LoggedNetworkNumber s = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kS", 0.375);
    public static final LoggedNetworkNumber v = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kV", 6);
    // Gravity feedforward: constant voltage to hold the arm's weight (Elevator_Static). Sign matters.
    public static final LoggedNetworkNumber g = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kG", 0.0);
    public static final double maxIAccum = 0.2;

    // Tunable target angles (mechanism rotations) the pov up/down buttons drive to.
    // Clamped by the soft limits below, so widen those if you need more travel.
    public static final LoggedNetworkNumber upSetpoint = new LoggedNetworkNumber("/Tuning/Intake/Pivot/upSetpoint", -0.31);
    public static final LoggedNetworkNumber downSetpoint = new LoggedNetworkNumber("/Tuning/Intake/Pivot/downSetpoint", 0.0);

    // Motion Magic profile (mechanism units): cruise velocity rot/s, acceleration rot/s^2.
    public static final LoggedNetworkNumber mmCruiseVelocity = new LoggedNetworkNumber("/Tuning/Intake/Pivot/mmCruiseVel", 1.5);
    public static final LoggedNetworkNumber mmAcceleration = new LoggedNetworkNumber("/Tuning/Intake/Pivot/mmAccel", 3);

    public static final double gearRatio = 45 / 1;

    public static final boolean invert = false;
    public static final boolean gravityType = false;
    public static final boolean breakType = false; 

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kAbsoluteEncoder;

    public static final boolean useFMaxRotation = true;
    public static final boolean useRMaxRotation = true;
    public static final Angle maxReverseRotation = Units.Rotation.of(-0.32); // up/retract stop; less negative = stops sooner (further from motors)
    public static final Angle maxFowardRotation = Units.Rotation.of(0);

    public static final Current stallLimit = Units.Amps.of(80);
    public static final Current supplyLimit = Units.Amps.of(60);

  }
}