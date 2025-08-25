package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDcontroller {
    private final double kp;
    private final double ki;
    private final double kd;
    private int target = 0;
    private int prevError = 0;
    private long prevMillis = 0;
    Telemetry telemetry;

    public PIDcontroller(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public void setTarget(int target) {
        this.target = target;
    }
    public double calculate(int measurement) {
        //proportional
        int error = target - measurement;

        double p = error * kp;

        //integral
        long now = System.currentTimeMillis();
        double timeChange = (now - prevMillis);
        double i = (error * timeChange) * ki;

        //derivative
        double d = kd * (error - prevError) /timeChange;
        prevMillis = now;
        prevError = error;

        double sum = Range.clip(p + i + d, -10000000, 100000) ;
        return sum;

    }


}