package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterMath {

    private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

    static {
      
        speedMap.put(1.3, 1650.0);
        speedMap.put(1.5, 1750.0);
        speedMap.put(1.6, 1850.0);
        speedMap.put(1.7, 1975.0);
        speedMap.put(1.8, 1895.0);
        speedMap.put(1.9, 2050.0);
        speedMap.put(2.0, 1895.0);
        speedMap.put(2.2, 2125.0);
        speedMap.put(2.3, 2125.0);
        speedMap.put(2.9, 2400.0);
    }

    public static double calculateSpeed(double distanceToHub) {
        return speedMap.get(distanceToHub);
    }

}
