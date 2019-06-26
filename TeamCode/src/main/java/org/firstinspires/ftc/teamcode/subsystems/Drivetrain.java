package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.VectorObject;

public class Drivetrain {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private static HardwareMap hardwareMap = null;
    private VectorObject robot;
    private double gyro;
    double prevX2 = 0;
    double prevY2 = 0;
    public static Drivetrain init( double initX, double initY, double r ){
        return new Drivetrain( initX, initY, r );
    }
    private Drivetrain( double initX, double initY, double r ){
        leftFront = hardwareMap.get( DcMotor.class, "leftFront" );
        rightFront = hardwareMap.get( DcMotor.class, "rightFront" );
        leftBack = hardwareMap.get( DcMotor.class, "leftFront" );
        rightBack = hardwareMap.get( DcMotor.class, "leftFront" );
        robot = new VectorObject( initX, initY, r );
        //GYRO TEMP
        gyro = 0;
    }
    public void drive( double x, double y, double r ){
        //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
        double lf, rf, lr, rr;
        double x2 = Math.cos(-gyro)*x - Math.sin(-gyro)*y;
        double y2 = Math.sin(-gyro)*x + Math.cos(-gyro)*y;
        x2 = ( Math.abs( x2 - prevX2 ) <= 0.1 ) ? x2 : prevX2 + ( 0.1 * Math.signum( x2 - prevX2 ) );
        y2 = ( Math.abs( y2 - prevY2 ) <= 0.1 ) ? y2 : prevY2 + ( 0.1 * Math.signum( y2 - prevY2 ) );
        double p = Math.hypot( x2, y2 );
        double rotateAngle = Math.atan2( y2, x2 ) - (Math.PI / 4);
        lf = p * Math.cos(rotateAngle) + r;
        rf = p * Math.sin(rotateAngle) - r;
        lr = p * Math.sin(rotateAngle) + r;
        rr = p * Math.cos(rotateAngle) -r;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
    }
}
