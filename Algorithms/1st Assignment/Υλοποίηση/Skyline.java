import java.lang.System;
import java.lang.Override;
import java.io.BufferedReader;
import java.io.FileReader;
import java.lang.Math;
import java.lang.Integer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;
import java.util.HashSet;


public class Skyline
{
    private static class Point implements Comparable<Point>
    {
        private int x;
        private int y;

        private int hash;


        public Point(int x, int y)
        {
            this.x = x;
            this.y = y;

            hash = 17;
            hash = 31 * hash + x;
            hash = 31 * hash + y;
        }

        public void setX(int x)
        {
            this.x = x;

            hash = 17;
            hash = 31 * hash + this.x;
            hash = 31 * hash + this.y;
        }

        public void setY(int y)
        {
            this.y = y;

            hash = 17;
            hash = 31 * hash + this.x;
            hash = 31 * hash + this.y;
        }

        public int getX() {
            return x;
        }

        public int getY() {
            return y;
        }

        public boolean dominates(Point other)
        {
            return this.x <= other.x && this.y <= other.y && (this.x != other.x || this.y != other.y);
        }

        public double distanceFromOrigin()
        {
            return Math.sqrt(x * x + y * y);
        }

        @Override
        public boolean equals(Object other)
        {
            if (this == other) {
                return true;
            }
            if (!(other instanceof Point)) {
                return false;
            }
            Point p = (Point)other;

            return this.x == p.x && this.y == p.y;
        }

        @Override
        public String toString() {
            return x + " " + y;
        }

        @Override
        public int hashCode() {
            return hash;
        }

        @Override
        public int compareTo(Point p) {
            return Integer.compare(this.x, p.x);
        }
    }

    public static ArrayList<Point> getSkyline(final Point[] points)
    {
        ArrayList<Point> skyline = new ArrayList<>();
        Set<Point> alreadyInSkyline = new HashSet<>();
        boolean isDominated;
        int i;
        int j;

        for (i = 0; i < points.length; ++i)
        {
            isDominated = false;

            for (j = 0; j < points.length; ++j)
            {
                if (points[i].equals(points[j])) {
                    continue;
                }

                if (points[j].dominates(points[i])) {
                    isDominated = true;
                    break;
                }
            }

            if (!isDominated && !alreadyInSkyline.contains(points[i])) {
                skyline.add(points[i]);
                alreadyInSkyline.add(points[i]);
            }
        }
        Collections.sort(skyline);

        return skyline;
    }


    public static void main(String[] args)
    {
        Point[] points;
        ArrayList<Point> skyline;
        

        if (args.length == 0) {
            System.err.println("Input Error: No command line arguement provided");
            return;
        }

        try (BufferedReader in = new BufferedReader(new FileReader(args[0])))
        {
            String line;
            String[] coordinates;
            int expected;
            int i = 0;
            int x;
            int y;

            line = in.readLine();

            if (line == null) {
                System.err.println("Input Error: Blank File (" + args[0] + ")");
                return;
            }

            expected = Integer.parseInt(line);
            points = new Point[expected];

            while ((line = in.readLine()) != null && i < expected)
            {
                coordinates = line.split(" ");

                if (coordinates.length != 2) {
                    System.err.println("Input Error: Wrong Input at file's line: " + (i + 2));
                    return;
                }
                x = Integer.parseInt(coordinates[0]);
                y = Integer.parseInt(coordinates[1]);

                points[i++] = new Point(x, y);
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
            return;
        }

        skyline = getSkyline(points);

        for (Point p : skyline) {
            System.out.println(p);
        }
    }
}
