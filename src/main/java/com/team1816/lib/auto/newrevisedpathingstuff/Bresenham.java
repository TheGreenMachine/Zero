package com.team1816.lib.auto.newrevisedpathingstuff;

public class Bresenham {
    // function for line generation using Bresenham's line drawing algorithm
    public static boolean drawLine(FieldMap map, int x1, int y1, int x2, int y2) {
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

    public static int[] drawPerpLine(FieldMap map, double startMinRadius, int x1, int y1, int x2, int y2) {
        int midPixelX = (x2-x1)/2+x1;
        int midPixelY = (y2-y1)/2+y1;

        int hold = midPixelX-(y2-y1);
        y2 = midPixelY+(x2-x1);
        x2 = hold;

        x1 = midPixelX;
        y1 = midPixelY;

        int dx, dy, e;
        int incx, incy, inc1, inc2;
        int posX, posY;
        int negX, negY;
        dx = x2 - x1;
        dy = y2 - y1;
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        incx = 1;
        if (x2 < x1) incx = -1;
        incy = 1;
        if (y2 < y1) incy = -1;
        posX = x1; posY = y1;
        negX = x1; negY = y1;
        if (dx > dy) {
            if(dist(posX, posY, midPixelX, midPixelY) > startMinRadius){
                if (!map.checkPixelHasObjectOrOffMap(posX, posY)) {
                    return new int[]{posX, posY};
                }
                if (!map.checkPixelHasObjectOrOffMap(negX, negY)) {
                    return new int[]{negX, negY};
                }
            }
            e = 2 * dy - dx;
            inc1 = 2 * (dy - dx);
            inc2 = 2 * dy;
            while (map.checkPixelOnMap(posX, posY) || map.checkPixelOnMap(negX, negY))
            {
                if (e >= 0)
                {
                    posY += incy;
                    negY -= incy;
                    e += inc1;
                }
                else
                    e += inc2;
                posX += incx;
                negX -= incx;
                if(dist(posX, posY, midPixelX, midPixelY) > startMinRadius){
                    if (!map.checkPixelHasObjectOrOffMap(posX, posY)) {
                        return new int[]{posX, posY};
                    }
                    if (!map.checkPixelHasObjectOrOffMap(negX, negY)) {
                        return new int[]{negX, negY};
                    }
                }
            }
        }
        else
        {
            if(dist(posX, posY, midPixelX, midPixelY) > startMinRadius){
                if (!map.checkPixelHasObjectOrOffMap(posX, posY)) {
                    return new int[]{posX, posY};
                }
                if (!map.checkPixelHasObjectOrOffMap(negX, negY)) {
                    return new int[]{negX, negY};
                }
            }
            e = 2 * dx - dy;
            inc1 = 2 * (dx - dy);
            inc2 = 2 * dx;
            while (map.checkPixelOnMap(posX, posY) || map.checkPixelOnMap(negX, negY))
            {
                if (e >= 0)
                {
                    posX += incx;
                    negX -= incx;
                    e += inc1;
                }
                else
                    e += inc2;
                posY += incy;
                negY -= incy;
                if(dist(posX, posY, midPixelX, midPixelY) > startMinRadius) {
                    if (!map.checkPixelHasObjectOrOffMap(posX, posY)) {
                        return new int[]{posX, posY};
                    }
                    if (!map.checkPixelHasObjectOrOffMap(negX, negY)) {
                        return new int[]{negX, negY};
                    }
                }
            }
        }
        System.out.println("Error in perping");
        return null;
    }

    public static boolean drawQuadrant(FieldMap map, int centerX, int centerY, double radius, boolean first, boolean second, boolean third, boolean fourth){
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

    public static boolean drawCircle(FieldMap map, int centerX, int centerY, double radius){
        return drawQuadrant(map, centerX, centerY, radius, true, true, true, true);
    }

    public static double dist(int x1, int y1, int x2, int y2){
        return dist((double)x1, (double)y1, (double)x2, (double)y2);
    }
    public static double dist(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2-x1, 2)+Math.pow(y2-y1, 2));
    }
}