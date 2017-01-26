using UnityEngine;
using System.Collections;

public class Orbit : MonoBehaviour
{
    public Transform target;
    Vector2 axis;
    float radius = 5.0f;
    Vector3 XYZ;
    public float sensitivity = 0.15f;
    public float zoomSpeed = 15f;
    public float smoothing = 15f;
    public float minZoom = 1.0f;
    public float maxZoom = 50.0f;
    public bool invert = true;

    void Start()
    {
        if (!target) Debug.LogError("There is no target.");
    }

    void FixedUpdate()
    {
        // If target is set
        if (target)
        {
            // Hold scroller to rotate
            if (Input.GetMouseButton(0))
            {
                // Mouse movement
                axis.x += Input.GetAxis("Mouse X") * sensitivity * (((invert ? 1 : 0) * 2) - 1);
                axis.y += Input.GetAxis("Mouse Y") * sensitivity * (((invert ? 1 : 0) * 2) - 1);

                // Clamp camera angle
                if (axis.y < 0.1f) axis.y = 0.1f;
                if (axis.y >= Mathf.PI - 0.1f) axis.y = 3.0f;
            }
            // Camera XYZ positions from target
            XYZ.x = radius * Mathf.Sin(axis.y) * Mathf.Sin(axis.x) + target.position.x;
            XYZ.z = radius * Mathf.Sin(axis.y) * Mathf.Cos(axis.x) + target.position.z;
            XYZ.y = radius * Mathf.Cos(axis.y) + target.position.y;

            // Scroll to change distance
            radius -= Input.GetAxis("Mouse ScrollWheel") * zoomSpeed;
            radius = Mathf.Clamp(radius, 2.0f, 50.0f);

            // Smooth camera movement, set to 0 for no smoothing
            if (smoothing < 0) smoothing = 0;
            if (smoothing > 0) transform.position = Vector3.Lerp(transform.position, new Vector3(XYZ.x, XYZ.y, XYZ.z), Time.deltaTime * smoothing);
            else transform.position = new Vector3(XYZ.x, XYZ.y, XYZ.z);

            // Camera always looks at target
            transform.LookAt(target);
        }
    }
}
