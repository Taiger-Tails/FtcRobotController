package org.firstinspires.ftc.teamcode.Datatypes;

public class Vector2d {
    public double x;
    public double z;

    public void add(Vector2d other) {
        this.x += other.x;
        this.z += other.z;
    }

    public void sub(Vector2d other) {
        this.x -= other.x;
        this.z -= other.z;
    }

    public void mul(Vector2d other) {
        this.x *= other.x;
        this.z *= other.z;
    }

    public void div(Vector2d other) {
        this.x /= other.x;
        this.z /= other.z;
    }

    public Vector2d(double x, double z) {
        this.x = x;
        this.z = z;
    }
}
