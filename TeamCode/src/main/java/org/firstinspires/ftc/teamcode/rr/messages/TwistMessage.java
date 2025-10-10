package org.firstinspires.ftc.teamcode.rr.messages;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;

public final class TwistMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public TwistMessage(Twist2d twist) {
        this.timestamp = System.nanoTime();
        this.x = twist.line.x;
        this.y = twist.line.y;
        this.heading = twist.angle;
    }
}

