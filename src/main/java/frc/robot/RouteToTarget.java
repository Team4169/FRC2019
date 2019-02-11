package frc.robot;

/**
 * Utility class to hold the vectors the describe the route the robot needs
 * to follow to get from its current location to a vision target.  The 
 * intention is that, when the robot arrives at the target, it will be aligned
 * properly (normal i.e. perpendicular to the target).
 */
public class RouteToTarget {


    /**
     * The vector directly from the camera lens to the target.
     */
    private final Vec2D targetDirectVec;

    /**
     * The vector from the robot, in the robot's current direction of travel, to the
     * target normal vector.
     */
    private final Vec2D interceptVec;

    /**
     * The vector from the intercept vector to the target, normal (i.e.
     * perpendicular) to the plane in which the target lies. The invariant is that :
     * m_normalVec + m_interceptVec = m_targetDirectVec
     */
    private final Vec2D normalVec;

    /**
     * Constructor given the three vectors
     * 
     * @param targetDirectVec Vector directly from cameral lens to target
     * @param interceptVec    Vector from robot to normal vec
     * @param normalVec       Vector from normal vec to target
     */

    public RouteToTarget(Vec2D targetDirectVec, Vec2D interceptVec, Vec2D normalVec) {
        this.targetDirectVec = targetDirectVec;
        this.interceptVec = interceptVec;
        this.normalVec = normalVec;
    }

    /**
     * @return the targetDirectVec
     */
    public Vec2D getTargetDirectVec() {
        return targetDirectVec;
    }

    /**
     * @return the interceptVec
     */
    public Vec2D getInterceptVec() {
        return interceptVec;
    }

    /**
     * @return the normalVec
     */
    public Vec2D getNormalVec() {
        return normalVec;
    }




}