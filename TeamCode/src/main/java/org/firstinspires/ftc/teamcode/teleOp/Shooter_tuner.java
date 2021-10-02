package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot20;

@Config
@Autonomous(group = "tuner")
public class Shooter_tuner extends LinearOpMode {
    Robot20 R = new Robot20();
    public static double t1 = 300;
    public static double t2 = 500;
    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, false);
        R.attachGamepads(gamepad1, gamepad2);
        R.mode = Robot20.WB_mode.CONTROL;

        waitForStart();

        R.ShooterPID.start();
        while (opModeIsActive()){
            R.module_control();
        }

        R.ShooterPID.interrupt();
//
//        R.velocityCheck.interrupt();
    }
    void back (double x, double p) {
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 <= x) {
            R.set_Power(-p, -p, -p, -p);
        }
        R.set_Power(0, 0, 0, 0);
    }
    void forward (double x){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 <= x){
            R.set_Power(0.15, 0.15, 0.15, 0.15);
        }
        R.set_Power(0, 0, 0, 0);
    }
}
