package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

  public enum ShooterModes {
    SHOOT_FAR(Units.RPM.of(3500), Units.Rotations.of(.75)),
    SHOOT_CLOSE(Units.RPM.of(2500), Units.Rotations.of(.1)),
    IDLE(Units.RPM.of(0), Units.Rotations.of(0)),
    REVERSE(Units.RPM.of(-500), Units.Rotations.of(0));

    public AngularVelocity speed;
    public Angle angle;

    ShooterModes(AngularVelocity speed, Angle angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

  public static class ShooterFlywheelConstants {
    public static final boolean attached = true;
    public static final int leaderId = 51;
    public static final int followerId = 52;

    public static final boolean inverted = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    public static final double p = 0.002000;
    public static final double i = 0;
    public static final double d = 0;

    public static LoggedNetworkNumber testp = new LoggedNetworkNumber("/Tuning/Shooter/P", 0.003);
    public static final LoggedNetworkNumber testi = new LoggedNetworkNumber("/Tuning/Shooter/I", 0);
    public static final LoggedNetworkNumber testd = new LoggedNetworkNumber("/Tuning/Shooter/D", 0.00002);
    public static final LoggedNetworkNumber testkS = new LoggedNetworkNumber("/Tuning/Shooter/kS", 0.375);
    public static final LoggedNetworkNumber testkV = new LoggedNetworkNumber("/Tuning/Shooter/kV", 0.0021);
    public static final LoggedNetworkNumber tuneDesiredSpeed = new LoggedNetworkNumber("/Tuning/Shooter/desiredSpeed",
        0);

    public static final double kS = 0;
    public static final double kV = 0.001000; // feedforward
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.RotorSensor;
    public static final FeedbackSensor feedbackSensorREV = FeedbackSensor.kPrimaryEncoder;
    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);

  }

  public static class ShooterAnglerConstants {
    public static final boolean attached = true;
    public static final int id = 53;
    public static final boolean inverted = false;
    public static final boolean breakType = true;
    public static final double gearRatio = 1 / 1;

    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.RotorSensor;

    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);
    public static final Current homingCurrent = Units.Amps.of(35);
    // public static final double maxForwardOutput = 1;
    // public static final double maxReverseOutput = 0.5;

    public static LoggedNetworkNumber anglerP = new LoggedNetworkNumber("/Tuning/Angler/P", 1.0);
    public static final LoggedNetworkNumber anglerI = new LoggedNetworkNumber("/Tuning/Angler/I", 0);
    public static final LoggedNetworkNumber anglerD = new LoggedNetworkNumber("/Tuning/Angler/D", 0);
    public static final LoggedNetworkNumber anglerkS = new LoggedNetworkNumber("/Tuning/Angler/kS", 0);
    public static final LoggedNetworkNumber anglerkV = new LoggedNetworkNumber("/Tuning/Angler/kV", 0);
    public static final LoggedNetworkNumber tuneDesiredPosition = new LoggedNetworkNumber(
        "/Tuning/Angler/desiredPosition", 0);

    public static final boolean tunePID = true;

    public static final BangBangController bangBangController = new BangBangController();

    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(2);

    public static final double tolerance = 0.1;
    public static LoggedNetworkNumber tunableTolerance = new LoggedNetworkNumber("/Tuning/Angler/tolerance", .1);
    public static LoggedNetworkNumber bangBangForwardVoltage = new LoggedNetworkNumber("/Tuning/Angler/bangBangForwardVoltage", 1);
    public static LoggedNetworkNumber bangBangReversedVoltage = new LoggedNetworkNumber("/Tuning/Angler/bangBangReversedVoltage", 1);
    public static LoggedNetworkNumber bangBangAngle = new LoggedNetworkNumber("/Tuning/Angler/bangBangAngle", 1);
  }
}
