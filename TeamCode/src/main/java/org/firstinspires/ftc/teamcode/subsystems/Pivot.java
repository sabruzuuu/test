package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDcontroller;

public class Pivot {
    private final DcMotor leftPivotMotor;
    private final DcMotor rightPivotMotor;
    private final DcMotor leftPulleyMotor;
    private final DcMotor rightPulleyMotor;
    private PIDcontroller rPulleyPiDcontroller;
    private PIDcontroller lPulleyPiDcontroller;
    private PIDcontroller rPivotPiDcontroller;
    private PIDcontroller lPivotPiDcontroller;
    private int pivotTarget = 0;
    Telemetry telemetry;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftPivotMotor = hardwareMap.get(DcMotor.class, "LP");
        leftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lPivotPiDcontroller = new PIDcontroller(0.03,0,0);
        //0.03, 0, 0
        rightPivotMotor = hardwareMap.get(DcMotor.class, "RP");
        rightPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rPivotPiDcontroller = new PIDcontroller(0.03,0,0);
        rightPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPulleyMotor = hardwareMap.get(DcMotor.class, "LPY");
        leftPulleyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPulleyMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPulleyMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightPulleyMotor = hardwareMap.get(DcMotor.class, "RPY");
        rightPulleyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPulleyMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //PID TARGET SETTING
    public void setPivotTarget (int targetPosition){
        pivotTarget = targetPosition;
        rPivotPiDcontroller.setTarget(pivotTarget);
        lPivotPiDcontroller.setTarget(pivotTarget);
    }
    public void pivotUpdate(){
        double pivotPower = rPivotPiDcontroller.calculate(leftPivotMotor.getCurrentPosition());
        rightPivotMotor.setPower(pivotPower);
        leftPivotMotor.setPower(pivotPower);
    }

    //GETTING + DISPLAYING POSITIONS
    public int getLeftPivotPosition() {
        return leftPivotMotor.getCurrentPosition();
    }
    public int getRightPivotPosition(){
        return rightPivotMotor.getCurrentPosition();
    }
    public int getLeftPulleyPosition(){
        return leftPulleyMotor.getCurrentPosition();
    }
    public int getRightPulleyPosition(){
        return rightPulleyMotor.getCurrentPosition();
    }
    public void displayPivotPostitions(){
        telemetry.addData("Left Pivot Position: ",getLeftPivotPosition());
        telemetry.addData("Right Pivot Position: ", getRightPivotPosition());
        telemetry.addData("Left Pivot Power: ", leftPivotMotor.getPower());
        telemetry.addData("Right Pivot Power: ", rightPivotMotor.getPower());
    }
    public void displayPulleyPositions(){
        telemetry.addData("Right Pulley Position: ", getRightPulleyPosition());
        telemetry.addData("Left Pulley Position: ", getLeftPulleyPosition());
        telemetry.addData("Right Pulley Power: ", rightPulleyMotor.getPower());
        telemetry.addData("Left Pulley Power: ", leftPivotMotor.getPower());
    }

    //MANUAL COMMANDS
    public void pivotUp () {
        leftPivotMotor.setPower(0.54);
        rightPivotMotor.setPower(0.54);
    }
    public void pivotDown () {
        leftPivotMotor.setPower(-0.54);
        rightPivotMotor.setPower(-0.54);
    }
    public void pivotStop () {
        leftPivotMotor.setPower(0);
        rightPivotMotor.setPower(0);
    }
    public void pulleyUp () {
        leftPulleyMotor.setPower(-1);
        rightPulleyMotor.setPower(-1);
    }
    public void pulleyDown () {
        leftPulleyMotor.setPower(1);
        rightPulleyMotor.setPower(1);
    }
    public void pulleyStop () {
        leftPulleyMotor.setPower(0);
        rightPulleyMotor.setPower(0);
    }
}

