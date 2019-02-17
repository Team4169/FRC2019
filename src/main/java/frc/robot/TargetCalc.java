package frc.robot;

public class TargetCalc{
    /**
     * Minimum rotation we'll deal with, in radians
     */
    public static final double MIN_ROTATE = 0.02;

    private final Limelight limeLight;

    public TargetCalc (Limelight l) {
        limeLight = l;
    }

    /**
     * Determine the "target vector" between the camera lens and the target identified
     * on the camera's screen at the camera pixel coordinates (px, py), given the
     * necessary information about the camera and the target.  The target vector is a
     * 2-dimensional vector, in the plane of the field, whose length is the floor distance
     * between the camera lens and the (floor projection of the) target, and whose 
     * direction is direction is the direction from the lens to the target.
     * We determine the angle by computing the angle from the center of the camera's
     * field of view to the horizontal component of the target pixel coordinates, and then
     * adding the robot's field angle.  We determine the distance by computing the angle
     * of the target above the horizontal (from the target's pixel location plus the mounting
     * angle of the camera), and then using the arctan of that angle together with the 
     * target height above the camera mount to calculate the distance.
     * @param robotVec Unit vector (field-relative) in the robot's current direction
     * @return Field-relative vector from camera lens to target
     */
    public Vec2D getTargetVector(Vec2D robotVec) {
        double targetDistance = limeLight.getDist(); // TODO might divide by zero
        System.out.println("targetDistance:" + targetDistance);

        /* (robot angle + X angle) and distance give us the target vector */
        Vec2D targetVector = Vec2D.makePolar(targetDistance, robotVec.getTheta() - limeLight.getTx());
        return targetVector;
    }
    /**
     * Calculate the desired route to the target identified on the camera's screen
     * at camera pixel coordinates (px, py), given the necessary information about
     * the robot and the target.  We need to know:
     *  - the robot's orientation on the field (specified as a unit vector, field-relative
     *    pointing in the direction the robot is pointing)
     *  - the target alignment, specified as a unit vector normal (perpendicular) to the
     *    target and pointing away from the target -- i.e. in the reflected-light direction
     *  - the target's height above the field
     *  - the desired "normal distance" -- i.e. the minimum distance we want the robot to be
     *    away from the target when it makes its final turn to drive perpendicular to the
     *    target to drop off its payload.
     * @param robotVec Unit vector (field-relative) in the robot's current direction
     * @param targNorm Unit vector (field-relative) pointing perpendicularly away from target
     * @param normDist Minimum distance from target (in units) for robot to drive normal to
     * target at end of its route
     * @return Route to target: vectors the robot must drive to get to the target from its
     * current position
     */

     public RouteToTarget getRouteToTarget(Vec2D robotVec, Vec2D targNorm, double normDist) {
         Vec2D targetVec = getTargetVector(robotVec);
         Vec2D normalVec = targNorm.scale(-normDist);
         Vec2D interceptVec = targetVec.subtract(normalVec);

         return new RouteToTarget(targetVec, interceptVec, normalVec);
     }
}