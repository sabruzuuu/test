package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Claw {
    private Servo diferentialOne;
    private Servo diferentialTwo;
    private Servo claw;
    private Telemetry telemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry){
        diferentialOne = hardwareMap.get(Servo.class,"dO");
        diferentialTwo = hardwareMap.get(Servo.class,"dT");
        claw = hardwareMap.get(Servo.class,"cl");
        this.telemetry = telemetry;
    }
    public void outakeposition(){
        diferentialOne.setPosition(0.8);
        diferentialTwo.setPosition(0.8);
    }
    public void observationposition(){
        diferentialOne.setPosition(0.5);
        diferentialTwo.setPosition(0.5);
    }
    public void intakeposition(){
        diferentialOne.setPosition(0);
        diferentialTwo.setPosition(0);
    }
    public void closeClaw(){
        claw.setPosition(-1);
    }
    public void openClaw(){
        claw.setPosition(1);
    }
    public double differentialOneServoPosition(){
        return diferentialOne.getPosition();
    }
    public double differentialTwoServoPosiotion(){
        return diferentialTwo.getPosition();
    }
    public void telemetryServoDisplay(){
        telemetry.addData("Differential 1 Position: ", differentialOneServoPosition());
        telemetry.addData("Differential 2 Position: ", differentialTwoServoPosiotion());
    }

}
