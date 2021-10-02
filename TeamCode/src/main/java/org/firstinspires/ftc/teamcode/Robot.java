package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    // insert your HardwareDevices with public modifier here

    public LinearOpMode li = null;
    public HardwareMap hwd = null;
    protected Telemetry tele = null;

    protected Gamepad opGamepad1 = null;
    protected Gamepad opGamepad2 = null;

    public Robot() {} // default constructor

    public void attachGamepads(Gamepad opGamepad1, Gamepad opGamepad2) {
        this.opGamepad1 = opGamepad1;
        this.opGamepad2 = opGamepad2;
    }

    public abstract void initHWD(); // link your real devices with HardwareDevices

    public abstract void init(LinearOpMode li, boolean usingRR); // initial values

    public final void delay(long milliseconds) {
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 < milliseconds){}
    }
}
