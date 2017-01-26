using UnityEngine;
using System.Collections;

public class Tire : PhysicsObject
{
    public bool canTurn = false;
    public float turnSpeed = 2.0f;

    public bool canTorque = true;
    public float enginePower = 150.0f;

    public PhysicsObject carBody;

    float turnAngle = 0;

    public override void SecondaryUpdate()
    {
        if (!transform.parent)
            return;
        
        // Only allow for right axis rotation type
        transform.parent.localEulerAngles = new Vector3(0, turnAngle, 0) + carBody.transform.localEulerAngles;
    }

    void Update()
    {
        // Lock position to the constraintObject
        if (physicsParent)
        {
            transform.position = constraintObject.position;
            position = transform.position;
        }


        TorqueControls();
        TurningControls();

    }

    void TorqueControls()
    {
        if (!canTorque)
            return;

        // The torque depends on the size of the tire
        float tireRadius = transform.localScale.x * 0.5f;
        float tirePower = tireRadius * enginePower;


        // Controls torque movement
        if (Input.GetKey(KeyCode.W))
            AddTorque(transform.right * tirePower);
        else if (Input.GetKey(KeyCode.S))
            AddTorque(-transform.right * tirePower);
    }

    void TurningControls()
    {
        if (!canTurn)
            return;

        // Controls turning movement
        if (Input.GetKey(KeyCode.A))
            turnAngle = Mathf.Lerp(turnAngle, -30.0f, Time.deltaTime * turnSpeed);
        else if (Input.GetKey(KeyCode.D))
            turnAngle = Mathf.Lerp(turnAngle, 30.0f, Time.deltaTime * turnSpeed);
        else
            turnAngle = Mathf.Lerp(turnAngle, 0.0f, Time.deltaTime * turnSpeed);
    }
}
