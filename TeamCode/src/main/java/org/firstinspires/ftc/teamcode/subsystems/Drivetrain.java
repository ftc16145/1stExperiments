package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.VectorObject;

public class Drivetrain {
    public enum driveType {
        mecanum, fourWheel
    }
    public enum fourWheelControl {
        tank, arcade
    }
    private fourWheelControl control;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private driveType type;
    private static HardwareMap hardwareMap = null;
    private VectorObject robot;
    private double gyro;
    double prevX2 = 0;
    double prevY2 = 0;
    public static Drivetrain init( double initX, double initY, double r, driveType type ){
        return new Drivetrain( initX, initY, r, type );

    }

    private Drivetrain( double initX, double initY, double r, driveType type ){
        leftFront = hardwareMap.get( DcMotor.class, "leftFront" );
        rightFront = hardwareMap.get( DcMotor.class, "rightFront" );
        leftBack = hardwareMap.get( DcMotor.class, "leftBack" );
        rightBack = hardwareMap.get( DcMotor.class, "rightBack" );
        if( type == driveType.fourWheel ){
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        } else {
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        }
        robot = new VectorObject( initX, initY, r );
        this.type = type;
        //GYRO TEMP
        gyro = 0;
    }
    public void setControl( fourWheelControl c ){
        control = c;
    }
    public void mecanumDrive( double x, double y, double r ){
        double lf, rf, lr, rr;
        double x2 = Math.cos(-gyro) * x - Math.sin(-gyro) * y;
        double y2 = Math.sin(-gyro) * x + Math.cos(-gyro) * y;
        x2 = (Math.abs(x2 - prevX2) <= 0.1) ? x2 : prevX2 + (0.1 * Math.signum(x2 - prevX2));
        y2 = (Math.abs(y2 - prevY2) <= 0.1) ? y2 : prevY2 + (0.1 * Math.signum(y2 - prevY2));
        double p = Math.hypot(x2, y2);
        double rotateAngle = Math.atan2(y2, x2);
        lf = p * Math.cos(rotateAngle) + r;
        rf = p * Math.sin(rotateAngle) - r;
        lr = p * Math.sin(rotateAngle) + r;
        rr = p * Math.cos(rotateAngle) - r;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
    }
    public void fourWheelTankDrive( double left, double right ){
        double lf, rf, lr, rr;
        lf = left;
        lr = left;
        rf= right;
        rr = right;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
    }
    public void fourWheelArcadeDrive( double x, double y ){
        double lf, rf, lr, rr;
        lf = y + x;
        lr = lf;
        rf = y - x;
        rr = rf;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lr);
        rightBack.setPower(rr);
    }
    public void stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
