using UnityEngine;
using System.Collections;

public class Test : MonoBehaviour
{
    Vector3 vToMC;
    Vector3 torque;
    Vector3 velocity;
    Vector3 angularVelocity;
    Vector3 angularInertia;
    float mass = 1.0f;
    float airResistance = 0.1f;
    float damping = 0.1f;

    void Awake()
    {
        // Assume all objects are cuboids and follow this law
        // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        // Only set inertia at start assuming they will not undergo scale transformations
        angularInertia = (mass / 12.0f) * new Vector3(Mathf.Pow(transform.localScale.x, 2) * Mathf.Pow(transform.localScale.z, 2),
                                                      Mathf.Pow(transform.localScale.y, 2) * Mathf.Pow(transform.localScale.z, 2),
                                                      Mathf.Pow(transform.localScale.x, 2) * Mathf.Pow(transform.localScale.y, 2));
    }

    void FixedUpdate()
    {
        AddForceAtPoint(new Vector3(1.0f, 1.0f, 0.0f), transform.position + transform.forward);
    }

    void AddForceAtPoint(Vector3 _force, Vector3 _worldPoint)
    {
        //Vector3 worldForcePoint = transform.position + transform.forward;
        Vector3 localForcePoint = _worldPoint - transform.position;

        ///////// Rotational force
        vToMC = _worldPoint - transform.position;
        torque = Vector3.Cross(vToMC, _force);

        // Movement force
        float angle = Vector3.Angle(vToMC, _force);
        // We project the force vector onto the vector to/from the center of mass
        // to get a resulting vector pointing in the direction that the object will move
        Vector3 vMC = vToMC;
        if (angle >= 90.0f)
            vMC *= -1.0f;
        //Vector3 movementForce = Vector3.Project(_force, (vMC + _force).normalized);
        Vector3 movementForce = Vector3.Project(_force, vMC.normalized);

        // Movement acceleration and velocity
        Vector3 acceleration = movementForce * (1.0f / mass);
        acceleration -= (velocity * damping);
        velocity += (acceleration - velocity * damping) * Time.fixedDeltaTime;

        // Angular acceleration and velocity
        // We add inertia for more realistic torque
        Vector3 angularAcceleration = new Vector3(torque.x / angularInertia.x, torque.y / angularInertia.y, torque.z / angularInertia.z);
        angularAcceleration -= (angularVelocity * damping);
        angularVelocity += angularAcceleration * Time.fixedDeltaTime - (angularVelocity * damping);

        ///////// Air resistance
        // Velocity air resistance
        Vector3 absVelocity = new Vector3(Mathf.Abs(velocity.x),
                                        Mathf.Abs(velocity.y),
                                        Mathf.Abs(velocity.z));
        Vector3 dragCoefficient = airResistance * new Vector3(absVelocity.x * velocity.x,
                                                            absVelocity.y * velocity.y,
                                                            absVelocity.z * velocity.z);
        velocity -= dragCoefficient * Time.fixedDeltaTime;

        // Angular velocity air resistance
        Vector3 absAngularVelocity = new Vector3(Mathf.Abs(angularVelocity.x),
                                                Mathf.Abs(angularVelocity.y),
                                                Mathf.Abs(angularVelocity.z));
        Vector3 angularDragCoefficient = airResistance * new Vector3(absAngularVelocity.x * angularVelocity.x,
                                                                    absAngularVelocity.y * angularVelocity.y,
                                                                    absAngularVelocity.z * angularVelocity.z);
        angularVelocity -= angularDragCoefficient * Time.fixedDeltaTime;

        // Translating to object
        transform.position += velocity * Time.fixedDeltaTime;
        transform.Rotate(angularVelocity * Time.fixedDeltaTime);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position + transform.forward, transform.position + transform.forward + Vector3.right);
        Gizmos.DrawLine(transform.position + transform.forward, transform.position + transform.forward + torque);
    }
}
// http://www.newtondynamics.com/forum/viewtopic.php?f=9&t=8032
// https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics