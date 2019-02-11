package frc.robot;

public class Vec2D {

    // The x-coordinate
    private double xCoord;

    // The y-coordinate
    private double yCoord;
     /**
     * The zero vector, for convenience
     */
    public static final Vec2D ZERO = makeCart(0.0, 0.0);
    
    /**
     * A constant defining the default distance at which two vectors are
     * considered "near", for use in the "isNear" method.
     */
    public static final double EPSILON = 0.00001d;


    public Vec2D(double xCoord, double yCoord) {
        this.xCoord = xCoord;
        this.yCoord = yCoord;
    }

    /**
     * Factory method to construct a vector given its cartesian coordinates
     * @param x The x-coord
     * @param y The y-coord
     * @return the vector
     */
    public static Vec2D makeCart(double x, double y) {
        return new Vec2D(x, y);
    }

    public static Vec2D makePolar(double r, double theta) {
        return new Vec2D(r * Math.cos(theta), r * Math.sin(theta));
    }


    /**
     * Add a vector, returning the result
     * @param a addend
     * @return The sum
     */
    public Vec2D add(Vec2D a) {
        return new Vec2D(this.xCoord + a.getXCoord(), this.yCoord + a.getYCoord());
    }

    /**
     * Subtract a vector, returning the result
     * @param s subtrahend
     * @return The difference
     */
    public Vec2D subtract(Vec2D s) {
        return new Vec2D(this.xCoord - s.getXCoord(), this.yCoord - s.getYCoord());
    } 

     /**
     * Multiply by a scalar (double), returning
     * the result.
     * @param m The scalar to multiply by
     * @return product of this vector and the scalar
     */
    public Vec2D scale(double scalar) {
        return new Vec2D(this.xCoord * scalar, this.yCoord * scalar);
    }

    //Getter for xCoord
    public double getXCoord() {
        return xCoord;
    }

    //Getter for yCoord
    public double getYCoord() {
        return yCoord;
    }

    //returns the length of the vector
    public double getR() {
        return Math.hypot(this.xCoord, this.yCoord);
    }

    //returns the angle of the vector (in radians)
    public double getTheta() {
        return Math.atan2(this.yCoord, this.xCoord) * 180 / Math.PI;
    }

    /**
    * Return true iff the specified vector is "near" this one.
    * "Nearness" is defined in terms of distance i.e. 
    *   abs(o - this) <= epsilon
    * In general vectors should be compared for nearness rather
    * than equality, because of floating-point rounding errors and
    * also because motion may result in inexact positioning.
    * @param o The other vector
    * @param epsilon The radius of nearness; two vectors are near if
    * their distance is less than or equal to epsilon
    * @return True iff o is near this vector
    */
    public boolean isNear(Vec2D vec, double epsilon) {
        // Quick check for exact equality
        if (vec.xCoord == xCoord && vec.yCoord == yCoord) {
            return true;
        }
        return (Math.hypot(vec.xCoord - xCoord, vec.yCoord - yCoord) <= epsilon);
    }
    /**
     * Return true iff the specified vector is "near" this one.
    * "Nearness" is defined in terms of distance i.e. 
    *   abs(o - this) <= Vec2d.EPSILON
    * In general vectors should be compared for nearness rather
    * than equality, because of floating-point rounding errors and
    * also because motion may result in inexact positioning.
    * @param o The other vector
    * @return True iff o is near this vector
    */
    public boolean isNear(Vec2D vec) {
        return isNear(vec, Vec2D.EPSILON);
    }

    @Override
    public boolean equals(Object o) {
        if(!(o instanceof Vec2D)) {
            return false;
        } else {
            Vec2D v = (Vec2D)o;
            return (v.xCoord == xCoord && v.yCoord == yCoord);  
        }
    }



}