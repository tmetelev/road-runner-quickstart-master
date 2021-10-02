package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontroller {
    public double kP, kI, kD, kF;
    private double P, I, D;
    private ElapsedTime t = new ElapsedTime();
    public double target;
    public double output;
    private double error, prevError, time, prevTime;
    private double errorSum;
    public double maxOutput;
    public boolean isOutputBounded = false;

    FtcDashboard dashboard;

    private void reset() {
        output = 0;
        P = 0;
        I = 0;
        D = 0;
        error = 0;
        prevError = 0;
        time = 0;
        prevTime = 0;
        errorSum = 0;
        target = 0;
        t = null;
    }

    public PIDcontroller( double kP, double kI, double kD, double kF)
    {
        this.kF = kF;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        t.startTime();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public double getAction( double current ) {
        error = target - current;
        P = kP * error;
        time = t.milliseconds();
        if (output < maxOutput || !isOutputBounded)
            errorSum += error * (time - prevTime);
        D = kD * (error - prevError) / (time - prevTime);
        I = kI * errorSum;
        prevTime = time;
        prevError = error;
        output = P + I + D + kF;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("output", output);
        packet.put("error", error);
        packet.put("current", current);
        dashboard.sendTelemetryPacket(packet);
        return output;
    }
}
