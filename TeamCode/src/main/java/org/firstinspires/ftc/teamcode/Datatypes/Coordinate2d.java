package org.firstinspires.ftc.teamcode.Datatypes;

public class Coordinate2d {
    public Vector2d Position;
    public double Rotation;

    public Coordinate2d(Vector2d Position, double Rotation) {
        this.Position = Position;
        this.Rotation = Rotation;
    }

    public Coordinate2d(Vector2d Position, Vector2d LookAt) {
        double DirectionX = LookAt.x - Position.x;
        double DirectionY = LookAt.z - Position.z;

        double Angle = Math.toRadians(Math.atan2(DirectionY, DirectionX));

        this.Position = Position;
        this.Rotation = Angle;
    }
}
