package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot20;

@TeleOp
public class TELE_SOLO extends LinearOpMode {
    Robot20 R = new Robot20();
    @Override
    public void runOpMode() throws InterruptedException {
        R.init(this, false);
        R.attachGamepads(gamepad1, gamepad2);

        waitForStart();

        R.modeW = Robot20.Wobble_mode.STOP;
        R.mode = Robot20.WB_mode.SOLO_CONTROL;
        R.WheelBase.start();
        R.WobbleControl.start();
        R.ShooterPID2.start();

        while (opModeIsActive()){
            if (Math.abs(gamepad1.right_stick_y) >= 0.1)
                R.modeW = Robot20.Wobble_mode.SOLO_CONTROL;

            R.module_control_solo();
            telemetry.addData("isShooting", R.flag_shoot);
            telemetry.addData("isTarget", R.target_shoot);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);
            telemetry.update();
        }

        R.WobbleControl.interrupt();
        R.WheelBase.interrupt();
        R.ShooterPID2.interrupt();
    }
}
