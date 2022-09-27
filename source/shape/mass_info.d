module shape.mass_info;

import math.mat33;

/**
 * This class holds mass information of a shape.
 * @author lo-th
 * @author saharan
 */


public class MassInfo {
    float mass;
    float inertia;

    this() {
        // Mass of the shape.
        this.mass = 0;

        // The moment inertia of the shape.
        this.inertia = new Mat33();
    }
}