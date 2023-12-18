package com.team1816.lib.auto.revisedpathingstuff;

public class Bresenham {
    // function for line generation using Bresenham's line drawing algorithm
    public static boolean drawLine(FieldMapTest map, int x1, int y1, int x2, int y2) {
        boolean collided = false;
        int dx, dy, i, e;
        int incx, incy, inc1, inc2;
        int x, y;
        dx = x2 - x1;
        dy = y2 - y1;
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        incx = 1;
        if (x2 < x1) incx = -1;
        incy = 1;
        if (y2 < y1) incy = -1;
        x = x1; y = y1;
        if (dx > dy) {
            collided = !map.drawPixel(x, y) || collided;
            e = 2 * dy - dx;
            inc1 = 2 * (dy - dx);
            inc2 = 2 * dy;
            for (i = 0; i < dx; i++)
            {
                if (e >= 0)
                {
                    y += incy;
                    e += inc1;
                }
                else
                    e += inc2;
                x += incx;
                collided = !map.drawPixel(x, y) || collided;
            }
        }
        else
        {
            collided = !map.drawPixel(x, y) || collided;
            e = 2 * dx - dy;
            inc1 = 2 * (dx - dy);
            inc2 = 2 * dx;
            for (i = 0; i < dy; i++)
            {
                if (e >= 0)
                {
                    x += incx;
                    e += inc1;
                }
                else
                    e += inc2;
                y += incy;
                collided = !map.drawPixel(x, y) || collided;
            }
        }
        return collided;
    }

    public static boolean drawQuadrant(FieldMapTest map, int centerX, int centerY, double radius, boolean first, boolean second, boolean third, boolean fourth){
        boolean collided = false;

        double d = 3.14 - (2 * radius);
        int x = 0;
        double y = radius;

        if(first) {
            collided = map.drawPixel(centerX + x, centerY + (int)y) || collided;
            collided = map.drawPixel(centerX + (int)y, centerY + x) || collided;
        }
        if(second) {
            collided = map.drawPixel(centerX - x, centerY + (int)y) || collided;
            collided = map.drawPixel(centerX - (int)y, centerY + x) || collided;
        }
        if(third) {
            collided = map.drawPixel(centerX - x, centerY - (int)y) || collided;
            collided = map.drawPixel(centerX - (int)y, centerY - x) || collided;
        }
        if(fourth) {
            collided = map.drawPixel(centerX + x, centerY - (int)y) || collided;
            collided = map.drawPixel(centerX + (int)y, centerY - x) || collided;
        }

        while (y >= x)
        {
            // for each pixel we will
            // draw all eight pixels

            x++;

            // check for decision parameter
            // and correspondingly
            // update d, x, y
            if (d > 0)
            {
                y--;
                d = d + 4 * (x - y) + 10;
            }
            else
                d = d + 4 * x + 6;

            if(first) {
                collided = map.drawPixel(centerX + x, centerY + (int)y) || collided;
                collided = map.drawPixel(centerX + (int)y, centerY + x) || collided;
            }
            if(second) {
                collided = map.drawPixel(centerX - x, centerY + (int)y) || collided;
                collided = map.drawPixel(centerX - (int)y, centerY + x) || collided;
            }
            if(third) {
                collided = map.drawPixel(centerX - x, centerY - (int)y) || collided;
                collided = map.drawPixel(centerX - (int)y, centerY - x) || collided;
            }
            if(fourth) {
                collided = map.drawPixel(centerX + x, centerY - (int)y) || collided;
                collided = map.drawPixel(centerX + (int)y, centerY - x) || collided;
            }
        }
        return collided;
    }

    public static boolean drawCircle(FieldMapTest map, int centerX, int centerY, double radius){
        return drawQuadrant(map, centerX, centerY, radius, true, true, true, true);
    }
}