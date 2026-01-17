// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
  public static final String canbus = "Omnivore";

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

    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    // public static final double maxIAccum = 2 * i; //CTRE Doesn't have one? Might Add later

    public static final Current stallLimit = Units.Amps.of(60);
    public static final Current supplyLimit = Units.Amps.of(80);
    public static final double maxForwardOutput = 1;
    public static final double maxReverseOutput = 0.5;

    public static final AngularVelocity shooterSpeed = Units.RPM.of(4000);
    public static final AngularVelocity reverseSpeed = Units.RPM.of(-1000); 
    public static final AngularVelocity idleSpeed = Units.RPM.of(0);

    public enum ShooterModes {
            SHOOTER(shooterSpeed, 1.0),
            REVERSE(reverseSpeed, -0.5),
            IDLE(idleSpeed, 0.0);

            public AngularVelocity speed;
            public double output;

            ShooterModes(AngularVelocity speed, double output) {
                this.speed = speed;
                this.output = output;
            }

        }


  }

  public static final class IntakeConstants {
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

    public static final FeedbackSensorSourceValue feedbackSensor =
        FeedbackSensorSourceValue.FusedCANcoder;

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

    public static final AngularVelocity intakeSpeed = Units.RPM.of(3000);
    public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
    public static final AngularVelocity idleSpeed = Units.RPM.of(0);

    public enum IntakeModes {
      IDLE(idleSpeed, 0.0),
      INTAKE(intakeSpeed, 0.7),
      OUTTAKE(outtakeSpeed, -0.4);

      public AngularVelocity speed;
      public double output;

      IntakeModes(AngularVelocity speed, double output) {
        this.speed = speed;
        this.output = output;
      }
    }
  }
}
