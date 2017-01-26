using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SpringAttachment : MonoBehaviour
{
    public PhysicsObject attachment;
    public PhysicsObject car;

    public float springConstant = 4.0f;
    public float springDamping = 0.5f;

    float initialDistance;
    float distance;

    Vector3 force;

    void Start()
    {
        initialDistance = Mathf.Abs(attachment.transform.position.y - transform.position.y);
    }

    void FixedUpdate()
    {
        // Calculate springforce F=-kx
        distance = Mathf.Abs(attachment.transform.position.y - transform.position.y) - initialDistance;
        Vector3 direction = (attachment.transform.position - transform.position).normalized;
        force = distance * springConstant * Vector3.down - (force * springDamping);

        float forceMagnitude = force.magnitude;
        forceMagnitude = Mathf.Clamp(forceMagnitude, -500, 500);
        force = force.normalized * forceMagnitude;

        // Add force to attached objects
        car.AddForceAtPoint(force, transform.position);
        attachment.AddForce(-force);
    }

    // Draw springs
    void OnDrawGizmos()
    {
        Gizmos.color = Color.magenta;
        if(attachment)
            Gizmos.DrawLine(attachment.transform.position, transform.position);
    }
}
