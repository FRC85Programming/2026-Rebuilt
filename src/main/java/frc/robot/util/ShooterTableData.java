package frc.robot.util;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ShooterTableData {
    @JsonProperty("points")
    private List<ShooterPoint> points;

    public ShooterTableData() {
        this.points = new ArrayList<>();
    }

    public ShooterTableData(List<ShooterPoint> points) {
        this.points = points;
    }

    public List<ShooterPoint> getPoints() {
        return points;
    }

    public void setPoints(List<ShooterPoint> points) {
        this.points = points;
    }

    public static ShooterTableData fromJson(String json) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        return mapper.readValue(json, ShooterTableData.class);
    }

    public static ShooterTableData fromFile(File file) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        return mapper.readValue(file, ShooterTableData.class);
    }

    public String toJson() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        return mapper.writerWithDefaultPrettyPrinter().writeValueAsString(this);
    }

    public void toFile(File file) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.writerWithDefaultPrettyPrinter().writeValue(file, this);
    }

    public record ShooterPoint(
        @JsonProperty("distance") double distance,
        @JsonProperty("hoodAngleDegrees") double hoodAngleDegrees,
        @JsonProperty("flywheelRPM") double flywheelRPM
    ) {}
}
