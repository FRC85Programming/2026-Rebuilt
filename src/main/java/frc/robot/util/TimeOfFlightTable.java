package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TimeOfFlightTable {
  private static final InterpolatingDoubleTreeMap tofMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Low variance - Good results :)
    tofMap.put(1.75, 1.6); 
    tofMap.put(2.78, 1.7); 
    tofMap.put(3.76, 1.8); 
    tofMap.put(4.73, 1.7); 
    tofMap.put(5.6, 1.6); 
  }

  public static Double getTimeOfFlight(double distanceMeters) {
    return tofMap.get(distanceMeters);
  }
}
