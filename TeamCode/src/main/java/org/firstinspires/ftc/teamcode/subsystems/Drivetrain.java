package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
// import com.qualcomm.hardware.bosch.BNO055IMU/BHI260IMU;

public class Drivetrain {
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;

    public Drivetrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "lF");
        rightFront = hardwareMap.get(DcMotor.class,"rF");
        leftBack = hardwareMap.get(DcMotor.class,"lB");
        rightBack = hardwareMap.get(DcMotor.class,"rB");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double vd, double td, double vt) {
        double sin, cos, max;
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        sin = Math.sin(td - Math.PI / 4);
        cos = Math.cos(td - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFrontPower = vd * cos / max - vt;
        rightFrontPower = vd * sin / max + vt;
        leftBackPower = vd * sin / max - vt;
        rightBackPower = vd * cos / max + vt;

        if ((vd + Math.abs(vt)) > 1) {
            leftFrontPower /= vd + vt;
            rightFrontPower /= vd + vt;
            leftBackPower /= vd + vt;
            rightBackPower /= vd + vt;
        }
        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
    private void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower){
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
    public void stop(){
        setMotorPowers(0,0,0,0);
    }

    public void slowmode(double vd, double td, double vt){
        double sin, cos, max;
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        sin = Math.sin(td - Math.PI / 4);
        cos = Math.cos(td - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFrontPower = vd * cos / max - vt;
        rightFrontPower = vd * sin / max + vt;
        leftBackPower = vd * sin / max - vt;
        rightBackPower = vd * cos / max + vt;

        if ((vd + Math.abs(vt)) > 1) {
            leftFrontPower /= vd + vt;
            rightFrontPower /= vd + vt;
            leftBackPower /= vd + vt;
            rightBackPower /= vd + vt;
        }
        setMotorPowers(leftFrontPower*0.6, rightFrontPower*0.6, leftBackPower*0.6, rightBackPower*0.6);
    }

}