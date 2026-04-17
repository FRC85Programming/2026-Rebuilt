package frc.robot.subsystems.webserver;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterTableData;
import frc.robot.util.ShooterTableManager;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

import java.nio.file.Paths;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

public class WebServer extends SubsystemBase {
    private final Javalin app;
    private final ShooterTableManager tableManager;
    private final DoubleSubscriber distanceSubscriber;
    private final AtomicInteger requestCount = new AtomicInteger(0);
    private final ObjectMapper mapper = new ObjectMapper();
    
    public WebServer() {
        tableManager = ShooterTableManager.getInstance();
        
        var ntInstance = NetworkTableInstance.getDefault();
        var table = ntInstance.getTable("SmartDashboard");
        distanceSubscriber = table.getDoubleTopic("DISTANCE TO HUB").subscribe(0.0);
        
        app = Javalin.create(config -> {
            config.staticFiles.add(
                Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "web").toString(),
                Location.EXTERNAL
            );
        });
        
        setupEndpoints();
        
        app.start(5800);
        System.out.println("WebServer started on port 5800");
        SmartDashboard.putString("WebServer/Status", "Running on port 5800");
    }
    
    private void setupEndpoints() {
        app.get("/api/shooter-data", ctx -> {
            try {
                ShooterTableData data = tableManager.getData();
                ctx.json(data);
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(500).result("Error retrieving shooter data: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.get("/api/current-distance", ctx -> {
            try {
                double distance = distanceSubscriber.get();
                ctx.json(Map.of("distance", distance));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(500).result("Error retrieving current distance: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.post("/api/update-point", ctx -> {
            try {
                Map<String, Object> body = mapper.readValue(ctx.body(), Map.class);
                
                double distance = ((Number) body.get("distance")).doubleValue();
                double hoodAngleDegrees = ((Number) body.get("hoodAngleDegrees")).doubleValue();
                double flywheelRPM = ((Number) body.get("flywheelRPM")).doubleValue();
                
                tableManager.updatePoint(distance, hoodAngleDegrees, flywheelRPM);
                
                SmartDashboard.putString("WebServer/LastUpdate", java.time.Instant.now().toString());
                System.out.println(String.format("Updated point: distance=%.2f, hood=%.1f, rpm=%.0f", 
                    distance, hoodAngleDegrees, flywheelRPM));
                
                ctx.status(200).json(Map.of("success", true));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(400).result("Error updating point: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.post("/api/update-rpm-point", ctx -> {
            try {
                Map<String, Object> body = mapper.readValue(ctx.body(), Map.class);
                
                double distance = ((Number) body.get("distance")).doubleValue();
                double rpm = ((Number) body.get("rpm")).doubleValue();
                
                tableManager.updateRPMPoint(distance, rpm);
                
                SmartDashboard.putString("WebServer/LastUpdate", java.time.Instant.now().toString());
                System.out.println(String.format("Updated RPM point: distance=%.2f, rpm=%.0f", 
                    distance, rpm));
                
                ctx.status(200).json(Map.of("success", true));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(400).result("Error updating RPM point: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.post("/api/update-angle-point", ctx -> {
            try {
                Map<String, Object> body = mapper.readValue(ctx.body(), Map.class);
                
                double distance = ((Number) body.get("distance")).doubleValue();
                double angle = ((Number) body.get("angle")).doubleValue();
                
                tableManager.updateAnglePoint(distance, angle);
                
                SmartDashboard.putString("WebServer/LastUpdate", java.time.Instant.now().toString());
                System.out.println(String.format("Updated angle point: distance=%.2f, angle=%.1f", 
                    distance, angle));
                
                ctx.status(200).json(Map.of("success", true));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(400).result("Error updating angle point: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.delete("/api/delete-rpm-point", ctx -> {
            try {
                Map<String, Object> body = mapper.readValue(ctx.body(), Map.class);
                double distance = ((Number) body.get("distance")).doubleValue();
                
                tableManager.deleteRPMPoint(distance);
                
                System.out.println(String.format("Deleted RPM point at distance=%.2f", distance));
                ctx.status(200).json(Map.of("success", true));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(400).result("Error deleting RPM point: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.delete("/api/delete-angle-point", ctx -> {
            try {
                Map<String, Object> body = mapper.readValue(ctx.body(), Map.class);
                double distance = ((Number) body.get("distance")).doubleValue();
                
                tableManager.deleteAnglePoint(distance);
                
                System.out.println(String.format("Deleted angle point at distance=%.2f", distance));
                ctx.status(200).json(Map.of("success", true));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(400).result("Error deleting angle point: " + e.getMessage());
                e.printStackTrace();
            }
        });
        
        app.post("/api/toggle-live", ctx -> {
            try {
                Map<String, Object> body = mapper.readValue(ctx.body(), Map.class);
                boolean enabled = (Boolean) body.get("enabled");
                
                tableManager.setLiveUpdatesEnabled(enabled);
                
                System.out.println("Live updates " + (enabled ? "enabled" : "disabled"));
                ctx.status(200).json(Map.of("success", true, "enabled", enabled));
                incrementRequestCount();
            } catch (Exception e) {
                ctx.status(400).result("Error toggling live updates: " + e.getMessage());
                e.printStackTrace();
            }
        });
    }
    
    private void incrementRequestCount() {
        int count = requestCount.incrementAndGet();
        SmartDashboard.putNumber("WebServer/RequestCount", count);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("WebServer/CurrentDistance", distanceSubscriber.get());
    }
    
    public void stop() {
        if (app != null) {
            app.stop();
            System.out.println("WebServer stopped");
        }
    }
}
