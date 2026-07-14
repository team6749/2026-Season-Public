package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldMap {
    private Map<String, Region> regions = new HashMap<>();
    private Map<String, Point> points = new HashMap<>();

    public static class Point {
        public final String id;
        private final Translation3d blueLocation;

        @JsonCreator
        public Point(@JsonProperty("id") String id,
                @JsonProperty("x") double x,
                @JsonProperty("y") double y,
                @JsonProperty("z") double z) {
            this.id = id;
            this.blueLocation = new Translation3d(x, y, z);
        }

        public Translation3d getFlippedLocation() {
            return AllianceFlipUtil.apply(blueLocation);
        }

    }

    public static class Region {
        public final String id;
        private final List<Translation2d> vertices;

        @JsonCreator
        public Region(@JsonProperty("id") String id,
                @JsonProperty("points") List<Map<String, Double>> points) {
            this.id = id;
            this.vertices = new ArrayList<>();
            for (Map<String, Double> p : points) {
                this.vertices.add(new Translation2d(p.get("x"), p.get("y")));
            }
        }

        public boolean contains(Translation2d point) {
            int i;
            int j;
            boolean result = false;

            List<Translation2d> verticesFlipped = new ArrayList<>();
            for (Translation2d p : vertices) {
                verticesFlipped.add(AllianceFlipUtil.apply(p));
            }

            for (i = 0, j = verticesFlipped.size() - 1; i < verticesFlipped.size(); j = i++) {
                if ((verticesFlipped.get(i).getY() > point.getY()) != (verticesFlipped.get(j).getY() > point.getY()) &&
                        (point.getX() < (verticesFlipped.get(j).getX() - verticesFlipped.get(i).getX())
                                * (point.getY() - verticesFlipped.get(i).getY())
                                / (verticesFlipped.get(j).getY() - verticesFlipped.get(i).getY())
                                + verticesFlipped.get(i).getX())) {
                    result = !result;
                }
            }
            return result;
        }
    }

    // Helper class for Jackson deserialization
    private static class FieldData {
        public List<Region> regions;
        public List<Point> points;
    }

    public FieldMap(String filename) {
        File file = new File(Filesystem.getDeployDirectory(), filename);
        if (file.exists()) {
            try {
                ObjectMapper mapper = new ObjectMapper();
                FieldData data = mapper.readValue(file, FieldData.class);

                for (Region r : data.regions) {
                    regions.put(r.id, r);
                }
                for (Point p : data.points) {
                    points.put(p.id, p);
                }

                System.out.println("Loaded FieldMap from " + filename + " with " + regions.size() + " regions and "
                        + points.size() + " points.");
            } catch (IOException e) {
                e.printStackTrace();
                System.err.println("Failed to load FieldMap from " + file.getAbsolutePath());
            }
        } else {
            System.err.println("FieldMap file not found: " + file.getAbsolutePath());
        }
    }

    public Point getPoint(String id) {
        return points.get(id);
    }

    public Region getRegion(String id) {
        return regions.get(id);
    }

}
