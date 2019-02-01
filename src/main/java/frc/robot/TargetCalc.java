package frc.robot;

public class TargetCalc{


    /**
     * The camera's horizontal field of view, in radians
     */
    private final double hFov;

    /**
     * The camera's vertical field of view, in radians
     */
    private final double vFov;

    /**
     * The camera's horizontal image resolution, in pixels
     */
    private final double hRes;

    /**
     * The camera's vertical image resolution, in pixels
     */
    private final double vRes;

    /**
     * The camera's lens height above the floor, in convenient units
     * (must be the same units as used in routeToTarget computations)
     */
    private final double cameraHeight;

    /**
     * The camera's vertical aiming angle relative to the horizon, in
     * radians (e.g. a camera pointing level with the floor will have an
     * aiming angle of 0)
     */
    private final double vAimAngle;

    /**
     * Minimum rotation we'll deal with, in radians
     */
    public static final double MIN_ROTATE = 0.02;

    /**
     * Constructor given the basic camera parameters
     * @param hFov Horizontal field of view (in degrees)
     * @param vFov Vertical field of view (in degrees)
     * @param hRes Horizontal resolution (in pixels)
     * @param vRes Vertical resolution (in pixels)
     * @param camHeight Camera height above floor
     * @param vAim Vertical aiming angle relative to horizon
     * (in degrees)
     */

    public TargetCalc (double hFov, double vFov, double hRes, double vRes, double cameraHeight, double vAimAngle) {
        this.hFov = Math.toRadians(hFov);
        this.vFov = Math.toRadians(vFov);
        this.hRes = hRes;
        this.vRes = vRes;
        this.cameraHeight = cameraHeight;
        this.vAimAngle = Math.toRadians(vAimAngle);
    }

    /**
     * Given pixel coordinates in the camera's view plane, with (0, 0) in the
     * upper left of the image and the plane resolution of m_hRes x m_vRes pixels
     * horizontally and vertically: compute and return a normalized vector from
     * the center of the image.  This is a simple coordinate transform to a new
     * normalized coordinate system in which the origin is at the center of the
     * camera's image, and the distance from the center to the edge of the image
     * in each direction is 1.0 normalized units.
     * @param px X-coordinate of pixel in view plane
     * @param py Y-coordinate of pixel in view plane
     * @return Normalized vector from origin at center of view plane to the pixel
     */
    public Vec2D getTargetPlaneVec(int px, int py) {

        /* For each direction, normalized coord is original coord - center coord
         * (to center the origin) divided by 1/2 width of view (to normalize).  For vertical
         * width, must also negate (since pixel coords increase downward).
         */
        double ctrX = (hRes - 1) / 2;
        System.out.println("ctrX = " + ctrX);
        double ctrY = (vRes - 1) / 2;
        System.out.println("ctrY = " + ctrY);
        double nx = ((double)px - ctrX) / (hRes / 2.0);
        double ny = (ctrY - (double)py) / (vRes / 2.0);
        System.out.println("(nx, ny): (" + nx + "," + ny + ")");

        /* Now, scale horiz and vertical coords to a common scale (pixels aren't
         * necessarily square because horiz and vert fields of view are different)
         */

        double scaledX = Math.tan(hFov / 2.0) * nx;
        double scaledY = Math.tan(vFov / 2.0) * ny;
        return Vec2D.makeCart(scaledX, scaledY);
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
     * @param px Camera x (horizontal) pixel coordinate of the target (0, 0 is upper left)
     * @param py Camera y (vertical) pixel coordinate of the target (0, 0 is upper left)
     * @param robotVec Unit vector (field-relative) in the robot's current direction
     * @param targHeight Height of target (in units) above the floor
     * @return Field-relative vector from camera lens to target
     */
    public Vec2D getTargetVector(int px, int py, Vec2D robotVec, double targHeight) {
        
        /* First, convert the (x,y) pixel coordinates to a normalized vector (from -1.0 to 1.0 in each
         * direction) on an imaginary view plane 1.0 unit in front of the camera.  This will allow us
         * to read off the horizontal and vertical angles from the camera to the target.
         */
        Vec2D targetPlaneVec = getTargetPlaneVec(px, py);

        /* Now, the 2-d vector from the image center to the target in the (vertical, normalized) target
         * plane has accounted for the differences in horizontal and vertical pixel resolution and field
         * of view of the camera; and it is normalized to 1.0 units from the camera.  So, the *central
         * angles* from the image center to the target are simply the arc tangents of the x and y
         * components of the vector.  Need those next.
         */

        System.out.println("target plane x:" + targetPlaneVec.getXCoord());
        System.out.println("target plane y:" + targetPlaneVec.getYCoord());

        double centralXAngle = Math.atan2(targetPlaneVec.getXCoord(), 1);
        double centralYAngle = Math.atan2(targetPlaneVec.getYCoord(), 1);

        System.out.println("centralXAngle:" + centralXAngle);
        System.out.println("centralYAngle:" + centralYAngle);

        /* Next, we can compute the distance to the target from the camera, based on the camera's height,
         * its aiming angle, and the central Y angle obtained above:
         *   distance = (targ ht - camera ht) / tan(camera angle + central y angle)
         * This formula is inaccurate if the target height and camera height are "too close" or the
         * sum of the camera angle and central y angle are too close to PI/2 radians (90 deg) -- so
         * robot designers should avoid those situations!
         */

         double targetDistance = (targHeight - cameraHeight) / Math.tan(vAimAngle + centralYAngle);
         System.out.println("targetDistance:" + targetDistance);

        /* (robot angle + X angle) and distance give us the target vector */
        Vec2D targetVector = Vec2D.makePolar(targetDistance, robotVec.getTheta() + centralXAngle);
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
     * @param px Camera x (horizontal) pixel coordinate of the target (0, 0 is upper left)
     * @param py Camera y (vertical) pixel coordinate of the target (0, 0 is upper left)
     * @param robotVec Unit vector (field-relative) in the robot's current direction
     * @param targNorm Unit vector (field-relative) pointing perpendicularly away from target
     * @param targHeight Height of target (in units) above the floor
     * @param normDist Minimum distance from target (in units) for robot to drive normal to
     * target at end of its route
     * @return Route to target: vectors the robot must drive to get to the target from its
     * current position
     */

     public RouteToTarget getRouteToTarget(int px, int py, Vec2D robotVec, Vec2D targNorm, double targHeight, double normDist) {
         Vec2D targetVec = getTargetVector(px, py, robotVec, targHeight);
         Vec2D normalVec = targNorm.scale(-normDist);
         Vec2D interceptVec = targetVec.subtract(normalVec);

         return new RouteToTarget(targetVec, interceptVec, normalVec);
     }
}