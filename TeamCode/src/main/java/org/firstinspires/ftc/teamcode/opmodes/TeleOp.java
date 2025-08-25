package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main TeleOp")
public class TeleOp extends LinearOpMode {
    private Drivetrain drivetrain;
    private Pivot pivot;
    private Claw claw;
    private boolean slowmode = false;
    private boolean ypress = false;
    private boolean holding = false;
    private boolean clawClosed = false;
    private boolean squarePressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
            drivetrain = new Drivetrain(hardwareMap);
            pivot = new Pivot(hardwareMap, telemetry);
            claw = new Claw(hardwareMap, telemetry);
            claw.closeClaw();
        }
        while (opModeIsActive()) {
            //INITIALIZING ALL CLASSES
            claw.telemetryServoDisplay();
            pivot.displayPivotPostitions();
            pivot.displayPulleyPositions();
            telemetry.update();

            //DRIVETRAIN CODE
            boolean ypresscurrent = gamepad1.y;
            if (ypresscurrent && !ypress) {
                slowmode = !slowmode;
            }

            ypress = ypresscurrent;

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double slowfactor = slowmode ? 0.35 : 1.0;

            double vd = Math.hypot(x, y) * slowfactor;
            double td = Math.atan2(y, x);
            double turnAdjusted = turn * slowfactor;

            drivetrain.drive(vd, td, turnAdjusted);


            //PIVOT CODE
            if (gamepad2.left_bumper) {
                pivot.pivotUp();
                holding = false;
            } else if (gamepad2.right_bumper) {
                pivot.pivotDown();
                holding = false;
            } else {
                if (!holding){
                    pivot.setPivotTarget(pivot.getLeftPivotPosition());
                    holding = true;
                }
                pivot.pivotUpdate();
            }

            // PULLEY/SLIDERS CODE
            if (gamepad2.dpad_up) {
                pivot.pulleyUp();
            } else if (gamepad2.dpad_down) {
                pivot.pulleyDown();
            } else {
                pivot.pulleyStop();
            }

            //CLAW CODE
            boolean squareCurrent = gamepad2.square;
            if (squareCurrent && !squarePressed) {
                clawClosed = !clawClosed;
                if (clawClosed) {
                    claw.closeClaw();
                } else {
                    claw.openClaw();
                }
            }
            squarePressed = squareCurrent;

            //DIFFERENTIAL CLAW CODE
            if (gamepad2.triangle){
                claw.intakeposition();
            }
            if (gamepad2.circle){
                claw.observationposition();
            }
            if (gamepad2.cross){
                claw.outakeposition();
            }
        }
    }
}