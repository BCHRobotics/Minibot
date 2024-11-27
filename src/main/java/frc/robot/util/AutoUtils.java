package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class AutoUtils {
    public static Transform2d applyRotationMatrix(Transform2d inputTransform, double angle) {
        double newX = inputTransform.getX() * Math.cos(angle * (Math.PI / 180)) + inputTransform.getY() * -Math.sin(angle * (Math.PI / 180));
        double newY = inputTransform.getX() * Math.sin(angle * (Math.PI / 180)) + inputTransform.getY() * Math.cos(angle * (Math.PI / 180));

        return new Transform2d(newX, newY, new Rotation2d());
    }

    // projecting a onto b
    public static Transform2d projectOntoVector(Transform2d a, Transform2d b) {
        double dot = dotProduct(a, b);
        double mag = magnitude(b);
        
        return b.times(dot / (mag * mag));
    }

    // Calculate the dot product of two vectors
    public static double dotProduct(Transform2d a, Transform2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    // Calculate the magnitude of a vector
    public static double magnitude(Transform2d a) {
        return Math.sqrt(a.getX() * a.getX() + a.getY() * a.getY());
    } 

    // get a NORMALIZED vector that represents robot direction
    public static Transform2d getHeadingVector(double heading) {
        return applyRotationMatrix(new Transform2d(1, 0, new Rotation2d()), heading);
    }
}
