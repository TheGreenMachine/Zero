package com.team1816.lib.auto.newpathingstuff;

import java.util.ArrayList;

public class Bresenham {
    // function for line generation using Bresenham's line drawing algorithm
    public static ArrayList<Pixel> draw_line(int x1, int x2, int y1, int y2) {
        ArrayList<Pixel> pixels = new ArrayList<>();
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
            pixels.add(new Pixel(x, y, 0, false));
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
                pixels.add(new Pixel(x, y, 0, false));
            }
        }
        else
        {
            pixels.add(new Pixel(x, y, 0, false));
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
                pixels.add(new Pixel(x, y, 0, false));
            }
        }
        return pixels;
    }
}
