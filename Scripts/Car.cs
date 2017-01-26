using UnityEngine;
using System.Collections;

public class Car : PhysicsObject
{
    public PhysicsObject tireFR;
    public PhysicsObject tireFL;
    public PhysicsObject tireBR;
    public PhysicsObject tireBL;

    void Start()
    {

    }

    float time = 0;

    public void Update()
    {
        time += Time.deltaTime;
        if (time > 1.0f)
        {
            Debug.Log(velocity);
            time = 0;
        }
    }

    public override void SecondaryUpdate()
    {
        transform.eulerAngles = new Vector3(0, transform.eulerAngles.y, 0);
        angularVelocity = new Vector3(0, angularVelocity.y, 0);

        // Add the force of the car to the children
        ShareForceToChildren();
    }

    void ShareForceToChildren()
    {
        Vector3 givenForce = force / 4.0f;

        tireFR.AddForce(givenForce);
        tireFL.AddForce(givenForce);
        tireBR.AddForce(givenForce);
        tireBL.AddForce(givenForce);
    }
}
