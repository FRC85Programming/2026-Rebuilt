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

    @JsonProperty("rpmPoints")
    private List<RPMPoint> rpmPoints;

    @JsonProperty("anglePoints")
    private List<AnglePoint> anglePoints;

    public ShooterTableData() {
        this.points = new ArrayList<>();
        this.rpmPoints = new ArrayList<>();
        this.anglePoints = new ArrayList<>();
    }

    public ShooterTableData(List<ShooterPoint> points) {
        this.points = points;
        this.rpmPoints = new ArrayList<>();
        this.anglePoints = new ArrayList<>();
    }

    public ShooterTableData(List<RPMPoint> rpmPoints, List<AnglePoint> anglePoints) {
        this.points = new ArrayList<>();
        this.rpmPoints = rpmPoints;
        this.anglePoints = anglePoints;
    }

    public List<ShooterPoint> getPoints() {
        return points;
    }

    public void setPoints(List<ShooterPoint> points) {
        this.points = points;
    }

    public List<RPMPoint> getRpmPoints() {
        return rpmPoints;
    }

    public void setRpmPoints(List<RPMPoint> rpmPoints) {
        this.rpmPoints = rpmPoints;
    }

    public List<AnglePoint> getAnglePoints() {
        return anglePoints;
    }

    public void setAnglePoints(List<AnglePoint> anglePoints) {
        this.anglePoints = anglePoints;
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

    public record RPMPoint(
        @JsonProperty("distance") double distance,
        @JsonProperty("rpm") double rpm
    ) {}

    public record AnglePoint(
        @JsonProperty("distance") double distance,
        @JsonProperty("angle") double angle
    ) {}
}
