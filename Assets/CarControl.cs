using UnityEngine;
using UnityEngine.UI;

public class CarControl : MonoBehaviour
{
    public float maxSteeringAngle = 30f;
    public float maxRPM = 10f;
    public float wheelRadius = 0.05f;
    public float maxMotorTorque = 300f;

    public Slider steeringSlider;
    public Slider throttleSlider;

    WheelControl[] wheels;
    Rigidbody rigidBody;

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        wheels = GetComponentsInChildren<WheelControl>();
        rigidBody.mass = 0.3f;

        // Set the slider range to match our steering angle range
        steeringSlider.minValue = -maxSteeringAngle;
        steeringSlider.maxValue = maxSteeringAngle;

    }

    void FixedUpdate()
    {
        float steeringAngle = steeringSlider.value;
        float throttleInput = throttleSlider.value;

        // Calculate target wheel angular velocity based on input RPM
        float targetAngularVelocity = throttleInput * maxRPM * 2 * Mathf.PI / 60f;
        
        // Calculate required torque to achieve target angular velocity
        float requiredTorque = CalculateRequiredTorque(targetAngularVelocity);

        foreach (var wheel in wheels)
        {
            if (wheel.steerable)
            {
                wheel.WheelCollider.steerAngle = steeringAngle;
            }
            
            if (wheel.motorized)
            {
                wheel.WheelCollider.motorTorque = requiredTorque;
                wheel.WheelCollider.brakeTorque = 0;
            }
        }
    }

    float CalculateRequiredTorque(float targetAngularVelocity)
    {
        float currentAngularVelocity = rigidBody.velocity.magnitude / wheelRadius;
        float angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / Time.fixedDeltaTime;
        float requiredTorque = angularAcceleration * rigidBody.mass * wheelRadius * wheelRadius;

        return Mathf.Clamp(requiredTorque, -maxMotorTorque, maxMotorTorque);
    }
}