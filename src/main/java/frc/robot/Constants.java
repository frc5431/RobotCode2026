// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // CAN bus that the devices are located on;
  public static final CANBus CANBUS = new CANBus("canivore", "./logs/example.hoot");

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ShooterConstants {

    public enum ShooterState {
      SHOOTING,
      REVERSE,
      IDLE
    }

    public static final boolean attached = true;
    public static final int id = 50;
    public static final boolean inverted = false;
    public static final boolean breakType = false;
    public static final double gearRatio = 1 / 1;

    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);
    public static final double maxForwardOutput = 1;
    public static final double maxReverseOutput = 0.5;
  }

  public static final class IntakeRollerIOConstants {
    public static final boolean attached = true;
    public static final boolean useRpm = false;

    public static final int id = -1;

    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    public static final double maxIAccum = 0.2;

    public static final double gearRatio = 1 / 1;

    public static final Current forwardTorqueLimit = Units.Amps.of(80);
    public static final Current reverseTorqueLimit = Units.Amps.of(80);

    public static final boolean invert = false;
    public static final boolean gravityType = false;
    public static final boolean breakType = false;

    public static final FeedbackSensorSourceValue feedbackSensorCTRE =
        FeedbackSensorSourceValue.FusedCANcoder;
    public static final FeedbackSensor feedbackSensorREV = 
        FeedbackSensor.kPrimaryEncoder;

    public static final double maxForwardOutput = 0.5;
    public static final double maxReverseOutput = -0.5;

    public static final boolean useFMaxRotation = true;
    public static final boolean useRMaxRotation = true;
    public static final Angle maxReverseRotation = Units.Rotation.of(-0.1);
    public static final Angle maxFowardRotation = Units.Rotation.of(5);

    public static final boolean useStallLimit = true;
    public static final boolean useSupplyLimit = true;
    public static final Current stallLimit = Units.Amps.of(80);
    public static final Current supplyLimit = Units.Amps.of(60);

    public static final AngularVelocity rollerIOSpeed = Units.RPM.of(3000);
    public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
    public static final AngularVelocity idleSpeed = Units.RPM.of(0);

    public enum RollerIOModes {
      IDLE(idleSpeed, 0.0),
      INTAKE(rollerIOSpeed, 8.4),
      OUTTAKE(outtakeSpeed, -4.8);

      public AngularVelocity speed;
      // TODO: Make sure voltage is just output * 12 (for 12V)
      public double voltage;

      RollerIOModes(AngularVelocity speed, double voltage) {
        this.speed = speed;
        this.voltage = voltage;
      }
    }
  }
}
