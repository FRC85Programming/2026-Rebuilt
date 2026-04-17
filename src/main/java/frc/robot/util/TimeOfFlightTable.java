package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TimeOfFlightTable {
  private static final InterpolatingDoubleTreeMap tofMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Low variance - Good results :)
    tofMap.put(1.75, 2.0); 
    tofMap.put(2.78, 2.37); 
    tofMap.put(3.76, 2.12); 
    tofMap.put(4.73, 2.2); 
    tofMap.put(5.6, 2.29); 
  }

  public static Double getTimeOfFlight(double distanceMeters) {
    return tofMap.get(distanceMeters);
  }
}
