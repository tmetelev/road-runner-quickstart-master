package org.firstinspires.ftc.teamcode.util;

public class WB {
    public static MotorPower merge(MotorPower translational, MotorPower rotational) {
        MotorPower merged = new MotorPower(0,0,0,0);
        double max = 0;
        for (int i = 0; i < 4; i++)
        {
            merged.powers[i] = translational.powers[i] + rotational.powers[i];
            if (Math.abs(merged.powers[i]) > max)
                max = Math.abs(merged.powers[i]);
        }

        if (max > 1)
            for (int i = 0; i < 4; i++)
                merged.powers[i] /= max;


        return merged;
    }


}
