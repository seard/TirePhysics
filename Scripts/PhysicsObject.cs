using UnityEngine;
using System.Collections;

public class PhysicsObject : MonoBehaviour
{
    public bool UsePhysics = true;
    public bool UseTraction = true;

    public PhysicsObject physicsParent;
    public Transform constraintObject;
    public Transform wheelHouseObject;

    public Vector3 scale;
    public float mass = 1.0f;
    public Vector3 gravity = new Vector3(0, -20.0f, 0);
    public float airResistance = 0.05f;
    public float bounciness = 0.1f;
    public float friction = 0.5f;
    public float kineticFriction = 0.3f;
    public float forceTransferSpeed = 1.0f;
    public float damping = 0.0025f;
    public float angularDamping = 0.001f;
    public float slippingLimit = 4.0f;

    public Vector3 position;
    public Vector3 rotation;

    public Vector3 velocity;
    public Vector3 angularVelocity;
    public Vector3 force;
    public Vector3 acceleration;
    public Vector3 angularAcceleration;
    public Vector3 torque;

    private Vector3 nextPosition;
    private Vector3 forceAddition;
    private Vector3 torqueAddition;
    private Vector3 inertia;
    private bool grounded;

    public virtual void SecondaryUpdate() { }

    void Awake()
    {
        // Initialize values
        if(scale.magnitude == 0)
            scale = transform.localScale;
        position = transform.position;
        rotation = transform.localEulerAngles;
        // Assume we won't change the object scale in runtime and therefore only calculate inertia once
        inertia = GetBoxInertia();
    }

    public void AddForce(Vector3 _f)
    {
        forceAddition += _f;
    }

    public void AddTorque(Vector3 _t)
    {
        torqueAddition += _t;
    }

    public void ClearVelocity()
    {
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;
    }

    void FixedUpdate()
    {
        if (!UsePhysics)
            return;

        // ---------- Applying forces ----------
        AddGravity(gravity);
        AddForceAddition();
        AddTorqueAddition();
        CalculateTraction();

        ShareParentForces();

        // ---------- Virtual function for use in derived classes ----------
        SecondaryUpdate();

        // ---------- Setting acceleration ----------
        CalculateAcceleration();
        CalculateAngularAcceleration();

        // ---------- Setting velocity ----------
        CalculateVelocity();
        CalculateAngularVelocity();

        // ---------- Adding air resistance ----------
        //AddAirResistance(airResistance);
        //AddAngularAirResistance(airResistance);

        // ---------- Update rigidbody position ----------
        UpdatePhysicsObject();

        // ---------- Check collision ----------
        if (CheckIfGrounded())
        {
            CalculateCollision();
        }

        // ---------- Reset variables ----------
        forceAddition = Vector3.zero;
        torqueAddition = Vector3.zero;
        force = Vector3.zero;
        torque = Vector3.zero;
        acceleration = Vector3.zero;
        angularAcceleration = Vector3.zero;
    }

    void ShareParentForces()
    {
        if (!physicsParent)
            return;

        // Get mass ratios
        float totalMass = mass + physicsParent.mass;
        float myMassRatio = mass / totalMass;
        float parentMassRatio = physicsParent.mass / totalMass;

        position = constraintObject.position;

        // The force transfered should equal the mass ratio between this object and the parent
        // A heavier parent should absorb more force than a lighter object would
        Vector3 givenForce = force * parentMassRatio;

        // We lose as much force as we give to the parent
        //force += givenForce;
        physicsParent.AddForceAtPoint(givenForce, constraintObject.position);
    }

    void ShareChildForces()
    {

    }

    public bool IsGrounded()
    {
        return grounded;
    }

    // Returns true if colliding
    bool CheckIfGrounded()
    {
        // Check next position for collision and update accordingly
        nextPosition = position + velocity * Time.fixedDeltaTime;

        // Check flat ground collision for next position update
        if (nextPosition.y - scale.y / 2.0f < 0.0f)
            grounded = true;
        else
            grounded = false;

        return grounded;
    }

    // Clamp total length of vector
    Vector3 ClampVector(Vector3 _v, float _min, float _max)
    {
        float magnitude = _v.magnitude;
        magnitude = Mathf.Clamp(magnitude, _min, _max);
        return _v.normalized * magnitude;
    }

    Vector3 lastPosition;

    // Transfers force to torque and vice versa
    void CalculateTraction()
    {
        if (!UseTraction)
            return;

        if (IsGrounded())
        {
            if (physicsParent)
            {
                transform.position = constraintObject.position;
            }

            // In order to get the actual movement velocity
            // needed to fix the fact that we set the tire positions
            // to their corresponding axis every frame
            Vector3 realVelocity = velocity;
            velocity = (transform.position - lastPosition) / Time.fixedDeltaTime;

            // In case you want to change input velocities
            Vector3 tmpVvelocity = velocity;
            Vector3 tmpAngularVelocity = angularVelocity;

            // Our wanted velocities
            Vector3 wantedVelocity = Vector3.Cross(tmpAngularVelocity, Vector3.up);
            Vector3 wantedAngularVelocity = Vector3.Cross(Vector3.up, tmpVvelocity);

            // Get the difference from wanted to current velocities
            Vector3 dV = wantedVelocity - tmpVvelocity;
            Vector3 dW = wantedAngularVelocity - tmpAngularVelocity;

                // Set errorMagnitude to the amount we are slipping
                // Input into the function (x/5) fór desired slipping threshold between 0 and 20
                float errorMagnitude = ((dW.magnitude - slippingLimit)/3.0f);
                // Only allow the value to exist between 0 and 1
                errorMagnitude = Mathf.Clamp(errorMagnitude, 0.0f, 1.0f);
                // Make friction move between static friction and kinetic friction 
                float realFriction = (errorMagnitude * kineticFriction) + ((1.0f - errorMagnitude) * friction);
                // Set friction depending on slipping
                float maxFrictionForce = -force.y * realFriction;

            // Divide by two to meet at equilibrium
            Vector3 eV = dV * 0.5f;
            Vector3 eW = dW * 0.5f;

            // Clamp forces to maxFrictionForce
            eV = ClampVector(eV * realFriction, -maxFrictionForce, maxFrictionForce);
            eW = ClampVector(eW * realFriction, -maxFrictionForce, maxFrictionForce);

            // Add traction force and torque
            force += eV * forceTransferSpeed / Time.fixedDeltaTime;
            torque += eW * forceTransferSpeed / Time.fixedDeltaTime;



            lastPosition = transform.position;
            velocity = realVelocity;
         }
    }
    
    void CalculateCollision()
    {
        // Simple point collision
        velocity.y *= (-bounciness);
        nextPosition.y = 0.0f + scale.y / 2.0f;
    }

    void AddGravity(Vector3 _gravity)
    {
        // Basic gravity acceleration
        force += mass * _gravity;
    }

    void AddForceAddition()
    {
        // Add the additional force
        force += forceAddition;
    }

    void AddTorqueAddition()
    {
        // Add the additional torque
        torque += torqueAddition;
    }

    void CalculateAcceleration()
    {
        // Acceleration divided by mass for F=ma
        acceleration = force / mass;
    }
    
    void CalculateVelocity()
    {
        // Update velocity by acceleration
        velocity += acceleration * Time.fixedDeltaTime - (SquaredVelocity() * damping);
    }

    void CalculateAngularAcceleration()
    {
        // Angular acceleration divided by mass for F=ma => T=Iw (mass should be inertia instead)
        angularAcceleration = torque / mass;
    }

    void CalculateAngularVelocity()
    {
        angularVelocity += angularAcceleration * Time.fixedDeltaTime - (SquaredAngularVelocity() * angularDamping);
    }

    Vector3 SquaredVelocity()
    {
        Vector3 absVelocity = new Vector3(Mathf.Abs(velocity.x),
                                          Mathf.Abs(velocity.y),
                                          Mathf.Abs(velocity.z)).normalized;
        Vector3 sqrVelocity = new Vector3(absVelocity.x * velocity.x,
                                          absVelocity.y * velocity.y,
                                          absVelocity.z * velocity.z);

        return sqrVelocity;
    }

    Vector3 SquaredAngularVelocity()
    {
        Vector3 absVelocity = new Vector3(Mathf.Abs(angularVelocity.x),
                                          Mathf.Abs(angularVelocity.y),
                                          Mathf.Abs(angularVelocity.z)).normalized;
        Vector3 sqrAngularVelocity = new Vector3(absVelocity.x * angularVelocity.x,
                                                 absVelocity.y * angularVelocity.y,
                                                 absVelocity.z * angularVelocity.z);

        return sqrAngularVelocity;
    }

    void AddAirResistance(float _airResistance)
    {
        // Velocity air resistance
        force -= _airResistance * SquaredVelocity() * Time.fixedDeltaTime;
    }

    void AddAngularAirResistance(float _airResistance)
    {
        // Angular velocity air resistance
        torque -= _airResistance * SquaredAngularVelocity() * Time.fixedDeltaTime;
    }

    void UpdatePhysicsObject()
    {
        // Update position
        position = nextPosition;
        transform.position = position;

        if (physicsParent)
            angularVelocity = Vector3.Project(angularVelocity, transform.parent.right);

        // Update rotations
        float angularMagnitude = angularVelocity.magnitude * Time.fixedDeltaTime * 3.1415f * 0.5f * transform.localScale.x;

        transform.RotateAround(angularVelocity.normalized, angularMagnitude);
    }

    // Convert bool to int
    static int ToInt32(bool _b)
    {
        if (_b == false)
            return 1;
        return 0;
    }

    public void AddForceAtPoint(Vector3 _force, Vector3 _worldPoint)
    {
        ///////// Rotational force
        Vector3 vToMC = _worldPoint - transform.position;
        Vector3 torque = Vector3.Cross(vToMC, _force);

        Vector3 movementForce = _force;

        if (vToMC.magnitude > 0.01f)
        {
            // Movement force
            float angle = Vector3.Angle(vToMC, _force);
            // We project the force vector onto the vector to/from the center of mass
            // to get a resulting vector pointing in the direction that the object will move
            Vector3 vMC = vToMC;
            if (angle >= 90.0f)
                vMC *= -1.0f;

            movementForce = Vector3.Project(_force, vMC.normalized);
        }

        AddForce(movementForce);
        AddTorque(torque);
    }

    public Vector3 ComputeForceAtPoint(Vector3 _force, Vector3 _worldPoint)
    {
        ///////// Rotational force
        Vector3 vToMC = _worldPoint - transform.position;

        Vector3 movementForce = _force;

        if (vToMC.magnitude > 0.01f)
        {
            // Movement force
            float angle = Vector3.Angle(vToMC, _force);
            // We project the force vector onto the vector to/from the center of mass
            // to get a resulting vector pointing in the direction that the object will move
            Vector3 vMC = vToMC;
            if (angle >= 90.0f)
                vMC *= -1.0f;
            //Vector3 movementForce = Vector3.Project(_force, (vMC + _force).normalized);
            movementForce = Vector3.Project(_force, vMC.normalized);
        }

        return movementForce;
    }

    public Vector3 ComputeTorqueAtPoint (Vector3 _force, Vector3 _worldPoint)
    {
        //Vector3 worldForcePoint = transform.position + transform.forward;
        Vector3 localForcePoint = _worldPoint - transform.position;

        ///////// Rotational force
        Vector3 vToMC = _worldPoint - transform.position;
        Vector3 torque = Vector3.Cross(vToMC, _force);

        return torque;
    }

    public Vector3 KineticEnergy()
    {
        Vector3 absVelocity = new Vector3(Mathf.Abs(velocity.x),
                                          Mathf.Abs(velocity.y),
                                          Mathf.Abs(velocity.z));

        Vector3 squaredVelocity = new Vector3(velocity.x * absVelocity.x,
                                              velocity.y * absVelocity.y,
                                              velocity.z * absVelocity.z);

        return 0.5f * mass * squaredVelocity; // KE = 1/2 * mV^2
    }

    public Vector3 RotationalKineticEnergy()
    {
        Vector3 absAngularVelocity = new Vector3(Mathf.Abs(angularVelocity.x),
                                          Mathf.Abs(angularVelocity.y),
                                          Mathf.Abs(angularVelocity.z));

        Vector3 squaredVelocity = new Vector3(angularVelocity.x * absAngularVelocity.x,
                                              angularVelocity.y * absAngularVelocity.y,
                                              angularVelocity.z * absAngularVelocity.z);

        return 0.333f * mass * squaredVelocity; // KEr = 1/3 * mV^2
    }

    Vector3 GetBoxInertia()
    {
        // Assume all objects are cuboids and follow this law
        // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        float sX = scale.x;
        float sY = scale.y;
        float sZ = scale.z;
        Vector3 i = (mass / 12.0f) * new Vector3(sX * sX + sZ * sZ,
                                                 sY * sY + sZ * sZ,
                                                 sX * sX + sY * sY);
        return i;
    }

    public virtual void SecondaryGizmos() { }

    // Draw velocity direction
    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(position, velocity.normalized * 3.0f);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(position, angularVelocity.normalized * 3.0f);

        // Virtual gizmo update for deriving classes
        SecondaryGizmos();
    }
}