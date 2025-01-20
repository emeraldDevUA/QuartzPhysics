package org.tourmaline.PlanePhysics;

import org.joml.Matrix3f;
import org.joml.Vector3f;
import org.tourmaline.RigidBody.RigidBody;

import java.util.List;

public class Plane extends RigidBody {

    private Engine engine;
    private final List<Wing> wings;
    public Plane(Matrix3f inertia, Vector3f position, float mass, Engine engine, List<Wing> wings) {
        super(inertia, position, mass);
        this.engine = engine;
        this.wings = wings;
    }

    @Override
    public void update(float dt){
        applyForceAtPoint(engine.getForce(this), new Vector3f(0));
        
        for(Wing wing: wings){
            Vector3f wingForce = wing.getForce(this);
        }

        super.update(dt);
    }
    public void setControlInput(float aileron, float elevator, float rudder){
        synchronized (wings) {
            wings.getFirst().setInputControl(aileron);
            wings.get(1).setInputControl(-aileron);
            wings.get(2).setInputControl(-elevator);
            wings.getLast().setInputControl(-rudder);
        }
    }




}
