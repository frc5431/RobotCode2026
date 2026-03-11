package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterMath {

    private static InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

    public static double calculateHoodPosition(double distanceToHub) {
        return hoodMap.get(distanceToHub);
    }

    public static double calculateSpeed(double distanceToHub) {
        return speedMap.get(distanceToHub);
    }



}
