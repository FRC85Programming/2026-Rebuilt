package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TimeOfFlightTable {
  private static final InterpolatingDoubleTreeMap tofMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Low variance - Good results :)
    tofMap.put(4.89, 1.315); 
    tofMap.put(4.05, 1.321); 
    tofMap.put(3.85, 1.29); 
    tofMap.put(3.32, 1.297); 
    tofMap.put(2.82, 1.315); 
    tofMap.put(2.29, 1.32); 
    tofMap.put(1.89, 1.32); 
  }

  public static Double getTimeOfFlight(double distanceMeters) {
    return tofMap.get(distanceMeters);
  }
}
