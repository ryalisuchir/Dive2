package org.firstinspires.ftc.teamcode.common.camera.kalman;

public interface Filter {
    double estimate(double measurement);
}