package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TimeOfFlightTable {
  private static final InterpolatingDoubleTreeMap tofMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Low variance - Good results :)
    tofMap.put(6.07, 1.3); 
    tofMap.put(3.2, 1.19); 
    tofMap.put(2.71, 1.12); 
    tofMap.put(2.41, 1.06); 
    tofMap.put(1.45, 0.96); 
  }

  public static Double getTimeOfFlight(double distanceMeters) {
    return tofMap.get(distanceMeters);
  }
}
