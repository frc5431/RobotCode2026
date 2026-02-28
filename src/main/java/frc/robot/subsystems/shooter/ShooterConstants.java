package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

  public enum ShooterModes {
    SHOOT_FAR(Units.RPM.of(5000), Units.Degree.of(30)),
    SHOOT_CLOSE(Units.RPM.of(2500), Units.Degree.of(60)),
    IDLE(Units.RPM.of(0), Units.Degree.of(0.0));

    public AngularVelocity speed;
    public Angle angle;

    ShooterModes(AngularVelocity speed, Angle angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

  public static class ShooterFlywheelConstants {
    public static final boolean attached = true;
    public static final int leaderId = 13;
    public static final int followerId = 14;

    public static final boolean inverted = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    // public static final double p = 0;
    // public static final double i = 0;
    // public static final double d = 0;

    public static final LoggedNetworkNumber testp = new LoggedNetworkNumber("Shooter/P", 0);
    public static final LoggedNetworkNumber testi = new LoggedNetworkNumber("Shooter/I", 0);
    public static final LoggedNetworkNumber testd = new LoggedNetworkNumber("Shooter/D", 0);
    public static final LoggedNetworkNumber testkS = new LoggedNetworkNumber("Shooter/kS", 0);
    public static final LoggedNetworkNumber testkV = new LoggedNetworkNumber("Shooter/kV", 0);

    // public static final double kS = 0.5;
    // public static final double kV = 0.14; //feedforward
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;

    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);

  }

  public static class ShooterAnglerConstants {
    public static final boolean attached = true;
    public static final int id = 15;
    public static final boolean inverted = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might
    // Add later

    public static final FeedbackSensorSourceValue feedbackSensorCTRE = FeedbackSensorSourceValue.FusedCANcoder;

    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);
    public static final Current homingCurrent = Units.Amps.of(50);
    // public static final double maxForwardOutput = 1;
    // public static final double maxReverseOutput = 0.5;

    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(2);

  }
}
