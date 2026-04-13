package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ShooterTableData.ShooterPoint;
import frc.robot.util.ShooterTableData.RPMPoint;
import frc.robot.util.ShooterTableData.AnglePoint;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class ShooterTableManager {
    private static ShooterTableManager instance;
    
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private final File jsonFile;
    private final File tempFile;
    
    private ShooterTableData data;
    private InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap;
    private InterpolatingDoubleTreeMap flywheelSpeedMap;
    private boolean liveUpdatesEnabled = true;
    
    private ShooterTableManager() {
        jsonFile = new File("/home/lvuser/deploy/shooter-table.json");
        tempFile = new File("/home/lvuser/deploy/shooter-table.json.tmp");
        
        loadFromJson();
        rebuildMaps();
        
        SmartDashboard.putString("ShooterTable/LastReload", java.time.Instant.now().toString());
        SmartDashboard.putNumber("ShooterTable/PointCount", data.getPoints().size());
        SmartDashboard.putBoolean("ShooterTable/LiveUpdatesEnabled", liveUpdatesEnabled);
    }
    
    public static synchronized ShooterTableManager getInstance() {
        if (instance == null) {
            instance = new ShooterTableManager();
        }
        return instance;
    }
    
    private void loadFromJson() {
        try {
            if (jsonFile.exists()) {
                data = ShooterTableData.fromFile(jsonFile);
                migrateOldFormat();
                if (data.getRpmPoints().isEmpty() && data.getAnglePoints().isEmpty()) {
                    System.out.println("Loaded shooter table from JSON: " + data.getPoints().size() + " points (old format)");
                } else {
                    System.out.println("Loaded shooter table from JSON: " + data.getRpmPoints().size() + " RPM points, " + data.getAnglePoints().size() + " angle points");
                }
            } else {
                System.out.println("JSON file not found, using hardcoded defaults");
                data = getDefaultData();
                migrateOldFormat();
                saveToJson();
            }
        } catch (IOException e) {
            System.err.println("Error loading shooter table JSON, using defaults: " + e.getMessage());
            data = getDefaultData();
            migrateOldFormat();
        }
    }
    
    private void migrateOldFormat() {
        if (data.getPoints() != null && !data.getPoints().isEmpty() && 
            (data.getRpmPoints() == null || data.getRpmPoints().isEmpty()) &&
            (data.getAnglePoints() == null || data.getAnglePoints().isEmpty())) {
            
            System.out.println("Migrating old format to new separate tables...");
            List<RPMPoint> rpmPoints = new ArrayList<>();
            List<AnglePoint> anglePoints = new ArrayList<>();
            
            for (ShooterPoint p : data.getPoints()) {
                rpmPoints.add(new RPMPoint(p.distance(), p.flywheelRPM()));
                anglePoints.add(new AnglePoint(p.distance(), p.hoodAngleDegrees()));
            }
            
            data.setRpmPoints(rpmPoints);
            data.setAnglePoints(anglePoints);
            data.setPoints(new ArrayList<>());
            
            saveToJson();
            System.out.println("Migration complete: " + rpmPoints.size() + " RPM points, " + anglePoints.size() + " angle points");
        }
    }
    
    private ShooterTableData getDefaultData() {
        List<ShooterPoint> points = new ArrayList<>();
        points.add(new ShooterPoint(1.45, 71, 3300.0));
        points.add(new ShooterPoint(2.41, 69, 3790.0));
        points.add(new ShooterPoint(2.71, 64, 3800.0));
        points.add(new ShooterPoint(3.2, 63, 3900.0));
        points.add(new ShooterPoint(3.6, 60, 4000.0));
        points.add(new ShooterPoint(3.8, 58, 4100.0));
        points.add(new ShooterPoint(4.28, 56, 4200.0));
        points.add(new ShooterPoint(4.53, 55, 4300.0));
        points.add(new ShooterPoint(4.63, 54, 4400.0));
        points.add(new ShooterPoint(5.13, 53, 4500.0));
        return new ShooterTableData(points);
    }
    
    public void rebuildMaps() {
        lock.writeLock().lock();
        try {
            hoodAngleMap = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), 
                Rotation2d::interpolate
            );
            flywheelSpeedMap = new InterpolatingDoubleTreeMap();
            
            for (AnglePoint point : data.getAnglePoints()) {
                hoodAngleMap.put(point.distance(), Rotation2d.fromDegrees(point.angle()));
            }
            
            for (RPMPoint point : data.getRpmPoints()) {
                flywheelSpeedMap.put(point.distance(), point.rpm());
            }
            
            SmartDashboard.putString("ShooterTable/LastReload", java.time.Instant.now().toString());
            SmartDashboard.putNumber("ShooterTable/RPMPointCount", data.getRpmPoints().size());
            SmartDashboard.putNumber("ShooterTable/AnglePointCount", data.getAnglePoints().size());
            System.out.println("Rebuilt shooter table maps with " + data.getRpmPoints().size() + " RPM points and " + data.getAnglePoints().size() + " angle points");
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    public void saveToJson() {
        lock.readLock().lock();
        try {
            data.toFile(tempFile);
            Files.move(tempFile.toPath(), jsonFile.toPath(), StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
            System.out.println("Saved shooter table to JSON");
        } catch (IOException e) {
            System.err.println("Error saving shooter table JSON: " + e.getMessage());
            e.printStackTrace();
        } finally {
            lock.readLock().unlock();
        }
    }
    
    public ShooterTable.ShooterSetpoint getSetpoint(double distanceMeters) {
        lock.readLock().lock();
        try {
            return new ShooterTable.ShooterSetpoint(
                hoodAngleMap.get(distanceMeters),
                flywheelSpeedMap.get(distanceMeters)
            );
        } finally {
            lock.readLock().unlock();
        }
    }
    
    public ShooterTableData getData() {
        lock.readLock().lock();
        try {
            return data;
        } finally {
            lock.readLock().unlock();
        }
    }
    
    public void updatePoint(double distance, double hoodAngleDegrees, double flywheelRPM) {
        updateRPMPoint(distance, flywheelRPM);
        updateAnglePoint(distance, hoodAngleDegrees);
    }
    
    public void updateRPMPoint(double distance, double rpm) {
        lock.writeLock().lock();
        try {
            List<RPMPoint> rpmPoints = data.getRpmPoints();
            boolean found = false;
            
            for (int i = 0; i < rpmPoints.size(); i++) {
                if (Math.abs(rpmPoints.get(i).distance() - distance) < 0.001) {
                    rpmPoints.set(i, new RPMPoint(distance, rpm));
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                rpmPoints.add(new RPMPoint(distance, rpm));
                rpmPoints.sort((a, b) -> Double.compare(a.distance(), b.distance()));
            }
            
            saveToJson();
            
            if (liveUpdatesEnabled) {
                rebuildMaps();
            }
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    public void updateAnglePoint(double distance, double angle) {
        lock.writeLock().lock();
        try {
            List<AnglePoint> anglePoints = data.getAnglePoints();
            boolean found = false;
            
            for (int i = 0; i < anglePoints.size(); i++) {
                if (Math.abs(anglePoints.get(i).distance() - distance) < 0.001) {
                    anglePoints.set(i, new AnglePoint(distance, angle));
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                anglePoints.add(new AnglePoint(distance, angle));
                anglePoints.sort((a, b) -> Double.compare(a.distance(), b.distance()));
            }
            
            saveToJson();
            
            if (liveUpdatesEnabled) {
                rebuildMaps();
            }
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    public void deleteRPMPoint(double distance) {
        lock.writeLock().lock();
        try {
            List<RPMPoint> rpmPoints = data.getRpmPoints();
            rpmPoints.removeIf(p -> Math.abs(p.distance() - distance) < 0.001);
            
            saveToJson();
            
            if (liveUpdatesEnabled) {
                rebuildMaps();
            }
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    public void deleteAnglePoint(double distance) {
        lock.writeLock().lock();
        try {
            List<AnglePoint> anglePoints = data.getAnglePoints();
            anglePoints.removeIf(p -> Math.abs(p.distance() - distance) < 0.001);
            
            saveToJson();
            
            if (liveUpdatesEnabled) {
                rebuildMaps();
            }
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    public void setLiveUpdatesEnabled(boolean enabled) {
        lock.writeLock().lock();
        try {
            this.liveUpdatesEnabled = enabled;
            SmartDashboard.putBoolean("ShooterTable/LiveUpdatesEnabled", enabled);
            
            if (enabled) {
                rebuildMaps();
            }
        } finally {
            lock.writeLock().unlock();
        }
    }
    
    public boolean isLiveUpdatesEnabled() {
        lock.readLock().lock();
        try {
            return liveUpdatesEnabled;
        } finally {
            lock.readLock().unlock();
        }
    }
}
