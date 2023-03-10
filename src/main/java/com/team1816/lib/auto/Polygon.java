package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Translation2d;

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

    public Polygon(Translation2d vertex1, Translation2d vertex2, Translation2d vertex3, Translation2d ... vertices) {
        ArrayList<Translation2d> vert = (ArrayList<Translation2d>) List.of(vertex1, vertex2, vertex3);
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
}
