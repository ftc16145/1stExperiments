package org.firstinspires.ftc.teamcode;

import static java.lang.Double.NaN;

public class VectorObject {
    double prevX;
    double prevY;
    double x;
    double y;
    double r;
    public VectorObject( double initX, double initY, double rotation ){
        x = initX;
        y = initY;
    }
    public void addVector( double x1, double y1, double b ){
        double rotateFix = Math.toRadians(-b);
        double x2 = Math.cos(rotateFix)*x1 - Math.sin(rotateFix)*y1;
        double y2 = Math.sin(rotateFix)*x1 + Math.cos(rotateFix)*y1;
        x += x2;
        y += y2;
    }
}
