package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot20;

@Config
@Disabled
@TeleOp
public class TELE_B extends LinearOpMode {
    Robot20 R = new Robot20();

    public static double LEFT_TARGET_ANG = -30;
    public static double MID_TARGET_ANG = -20;
    public static double RIGHT_TARGET_ANG = -10;
    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, false);
        R.attachGamepads(gamepad1, gamepad2);

        waitForStart();

        R.modeW = Robot20.Wobble_mode.STOP;
        R.mode = Robot20.WB_mode.CONTROL;
        R.WheelBase.start();
        R.WobbleControl.start();
        R.odometry.start();
//        R.getShooterVelocity.start();
//        R.velocityCheck.start();
        R.ShooterPID2.start();
        Robot20.statr_ang += 90;
        Robot20.current_pose_static = new Pose2d(0, 48, 0);

        //R.ve+ cityCheck.start();

        boolean flag = true;
        double target_ang;

        while (!isStopRequested()){
            target_ang = Robot20.statr_ang + R.getAngToGoalBlue();

            if (gamepad2.dpad_down){
                R.modeW = Robot20.Wobble_mode.DOWN;
                R.delay(300);
            } else if (gamepad2.dpad_right){
                R.modeW = Robot20.Wobble_mode.MID;
                R.delay(300);
            } else if (gamepad2.dpad_up){
                R.modeW = Robot20.Wobble_mode.UP;
                R.delay(300);
            } else if (gamepad2.dpad_left){
                R.modeW = Robot20.Wobble_mode.STOP;
                R.delay(300);
            } else if (Math.abs(gamepad2.right_stick_y) >= 0.1)
                R.modeW = Robot20.Wobble_mode.CONTROL;
            if (gamepad1.dpad_down){
                R.field_rotate = !R.field_rotate;
                flag = !flag;
                R.delay(300);
            }

            if (gamepad2.x) {
                R.wb_rotate = true;
                R.field_rotate = true;
                R.delay(150);
                R.target_angle = 90;
                R.delay(150);
            }
            else if (gamepad2.y) {
                R.wb_rotate = true;
                R.field_rotate = true;
                R.delay(150);
                R.target_angle = (float) target_ang;
                Robot20.statr_ang += (gamepad1.left_trigger * 0.25 - gamepad1.right_trigger * 0.25);
                R.delay(150);
            }
            else if (flag) {
                R.wb_rotate = false;
                R.field_rotate = false;
            }

            if (gamepad1.dpad_right) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = (float) RIGHT_TARGET_ANG;
                R.delay(150);
            }
            if (gamepad1.dpad_up) {
                R.wb_rotate = true;
                R.delay(150);
                R.target_angle = (float) MID_TARGET_ANG;
                R.delay(150);
            }
            if (gamepad1.dpad_left) {
//                R.wb_rotate = true;
//                R.delay(150);
//                R.target_angle = LEFT_TARGET_ANG;
//                R.delay(150);
                strafe(7, 0.5);
            }
            if (gamepad1.right_stick_button) {
                Robot20.statr_ang = -R.heading() + 90;
                R.delay(300);
            }
//            if (Math.abs(gamepad1.left_stick_x) >= 0.05 || Math.abs(gamepad1.left_stick_y) >= 0.05 || gamepad1.left_bumper || gamepad1.right_bumper ||
//                    gamepad1.right_trigger >= 0.05 || gamepad1.left_trigger >= 0.05){
//                R.wb_rotate = false;
//            }


            R.module_control();
//            R.odo.update();
//            R.current_pose = new Pose2d(R.odo.currentPose.getX() + Robot20.current_pose_static.getX(), R.odo.currentPose.getY() + Robot20.current_pose_static.getY());
//            telemetry.addData("WB_mode (field rotate)", R.field_rotate);
//            telemetry.addData("WB_mode (wb rotate)", R.wb_rotate);
//            telemetry.addData("Wobble_mode", R.modeW);
            telemetry.addData("isShooting", R.flag_shoot);
            telemetry.addData("isTarget", R.target_shoot);
//            telemetry.addData("IMU", R.heading() - Robot20.statr_ang);
//            telemetry.addData("x", R.current_pose.getX());
//            telemetry.addData("y", R.current_pose.getY());
//            telemetry.addData("Target", target_ang);
            telemetry.update();
        }
        R.WobbleControl.interrupt();
        R.WheelBase.interrupt();
        R.ShooterPID2.interrupt();
        Robot20.statr_ang = 0;
       // R.odometry.interrupt();
//        R.velocityCheck.interrupt();
//        R.getShooterVelocity.interrupt();
        //R.velocityCheck.interrupt();
    }

    void strafe (double x, double p){
        R.WheelBase.interrupt();
        //odo.update();
        x = R.current_pose.getY() - x;
        while (R.current_pose.getY() >= x){
            R.set_Power(p, -p, p, -p);
            //drive.update();
        }
        R.set_Power(0, 0, 0, 0);
//        rotate(R.c, 0.4);
        R.WheelBase.start();
    }

    void rotate (double x, double p){
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        p *= x / Math.abs(x);
        while (R.heading() >= x){
            R.set_Power(-p, -p, p, p);
        }
        R.set_Power(0, 0, 0, 0);
    }
}
