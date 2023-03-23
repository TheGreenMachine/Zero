package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Translation2d;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Defines a cartesian polygon based on a list of vertices
 */
public class Polygon {
    private List<Translation2d> vertices;

    public Polygon(List<Translation2d> vertices) {
        this.vertices = vertices;
    }

    public Polygon(Translation2d vertex1, Translation2d vertex2, Translation2d vertex3, Translation2d... vertices) {
        ArrayList<Translation2d> vert = new ArrayList<>();
        vert.add(vertex1);
        vert.add(vertex2);
        vert.add(vertex3);
        vert.addAll(Arrays.asList(vertices));
        this.vertices = vert;
    }

    public List<Translation2d> getVertices() {
        return vertices;
    }

    public void setVertices(List<Translation2d> vertices) {
        this.vertices = vertices;
    }

    /**
     * Utilizes a simple ray-casting planar geometric PIP algorithm to determine whether a point is contained by the polygon
     *
     * @param point - cartesian translation2d
     * @return true if point is contained by polygon
     */
    public boolean contains(Translation2d point) {
        if (vertices.contains(point))
            return false;

        boolean contained = false;

        for (int i = 0, j = vertices.size() - 1; i < vertices.size(); i++) {
            // checks if ray crosses edge of polygon
            if (
                ((vertices.get(i).getY() > point.getY()) != (vertices.get(j).getY() > point.getY())) // contained by y
                    && (point.getX() < (vertices.get(j).getX() - vertices.get(i).getX()) * (point.getY() - vertices.get(i).getY()) / (vertices.get(j).getY() - vertices.get(i).getY()) + vertices.get(i).getX()) // contained by x
            ) {
                // inverting boolean as ray crosses edge
                contained = !contained;
            }
            j = i;
        }
        return contained;
    }

    /**
     * Determines if a line between two points is contained by the polygon to any degree
     */
    public boolean intersects(Translation2d p1, Translation2d p2) {
        for (int i = 0; i < vertices.size(); i++) {
            if (((p1 == vertices.get(i) && p2 == vertices.get((i + 1) % vertices.size())) || (p2 == vertices.get(i) && p1 == vertices.get((i + 1) % vertices.size())))) {
                return false;
            }
        }

        double xIncrement, yIncrement;

        if (Math.abs(p2.getX() - p1.getX()) < Math.abs(p2.getY() - p1.getY())) {
            xIncrement = 0.01 * Math.signum(p2.getX() - p1.getX()); // approximate resolution to check slopes NOTE: might need to reduce resolution for finer calculations
            yIncrement = (p2.getY() - p1.getY()) / (p2.getX() - p1.getX()) * Math.abs(xIncrement) * Math.signum(p2.getY() - p1.getY());
        } else {
            yIncrement = 0.01 * Math.signum(p2.getY() - p1.getY()); // approximate resolution to check slopes NOTE: might need to reduce resolution for finer calculations
            xIncrement = (p2.getX() - p1.getX()) / (p2.getY() - p1.getY()) * Math.abs(yIncrement) * Math.signum(p2.getX() - p1.getX());
        }

        for (int i = 0; i < Math.max(Math.abs((p2.getX() - p1.getX()) / (2 * xIncrement)), Math.abs((p2.getY() - p1.getY()) / (2 * yIncrement))); i++) {
            if (
                this.contains(new Translation2d(p1.getX() + xIncrement * i, p1.getY() + yIncrement * i)) ||
                    this.contains(new Translation2d(p2.getX() - xIncrement * i, p2.getY() - yIncrement * i))
            ) {
                return true;
            }
        }
        return false;
    }

    /**
     * Utilizes ray-casting to determine if a line intersects the polygon
     */
    public boolean intersects(Line2D line) {
        return intersects(new Translation2d(line.getX1(), line.getY1()), new Translation2d(line.getX2(), line.getY2()));
    }
}
