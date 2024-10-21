using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using UnityEngine.UI;

public class CarAgent : Agent
{
    public float maxSteeringAngle = 30f;
    public float maxRPM = 15f;
    public float wheelRadius = 0.05f;
    public float maxMotorTorque = 300f;

    public Transform[] checkpoints;
    private int currentCheckpoint = 0;

    private Rigidbody rigidBody;
    private WheelControl[] wheels;
    private float lastWallCollisionTime = -5f;
    private float highScoreTime = 0f;
    public Slider steeringSlider;
    public Slider throttleSlider;
    private Vector3 startPosition;
    private Quaternion startRotation;

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        wheels = GetComponentsInChildren<WheelControl>();
        rigidBody.mass = 0.3f;

        startPosition = transform.position;
        startRotation = transform.rotation;

                // Set the slider range to match our steering angle range
        if (steeringSlider != null)
        {
            steeringSlider.minValue = 0;
            steeringSlider.maxValue = 60f;
            steeringSlider.value = 30f;
        }
        if (throttleSlider != null)
        {
            throttleSlider.minValue = 0;
            throttleSlider.maxValue = 60f;
        }

    }

    public override void OnEpisodeBegin()
    {
        ResetCar();
    }

    private void ResetCar()
    {
        transform.position = startPosition;
        transform.rotation = startRotation;
        rigidBody.velocity = Vector3.zero;
        rigidBody.angularVelocity = Vector3.zero;
        currentCheckpoint = 0;
        lastWallCollisionTime = -5f;
        highScoreTime = 0f;

        foreach (var checkpoint in checkpoints)
        {
            checkpoint.gameObject.SetActive(true);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Collect observations from RayPerception3D sensor
        // Assuming you have set up a RayPerceptionSensorComponent3D in the Unity Inspector
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get discrete actions
        int steeringAction = actions.DiscreteActions[0];
        int throttleAction = actions.DiscreteActions[1];

        // Convert discrete actions to continuous values
        float steeringAngle = Mathf.Lerp(-maxSteeringAngle, maxSteeringAngle, (float)steeringAction / 60f);
        float throttleInput = Mathf.Lerp(0f, 1f, (float)throttleAction/60f);

        // Calculate target wheel angular velocity based on input RPM
        float targetAngularVelocity = throttleInput * maxRPM * 300 * Mathf.PI / 60;

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

        // Apply rewards and penalties
        AddReward(-0.01f); // -1 point per second

        // Check for high score
        if (GetCumulativeReward() > 100f)
        {
            highScoreTime += Time.fixedDeltaTime;
            if (highScoreTime > 5f)
            {
                EndEpisode();
            }
        }
        else
        {
            highScoreTime = 0f;
        }
    }

    float CalculateRequiredTorque(float targetAngularVelocity)
    {
        float currentAngularVelocity = rigidBody.velocity.magnitude / wheelRadius;
        float angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / Time.fixedDeltaTime;
        float requiredTorque = angularAcceleration * rigidBody.mass * wheelRadius * wheelRadius;

        return Mathf.Clamp(requiredTorque, -maxMotorTorque, maxMotorTorque);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            if (Time.time - lastWallCollisionTime > 0.5f) // Prevent multiple penalties for the same collision
            {
                AddReward(-100f);
                lastWallCollisionTime = Time.time;
                EndEpisode();
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("checkpoint"))
        {
            if (checkpoints[currentCheckpoint].gameObject == other.gameObject)
            {
                float reward = 25f;
                if (currentCheckpoint == 1) reward = 50f;
                else if (currentCheckpoint == 2) reward = 75f;
                else if (currentCheckpoint == 3) reward = 100f;

                AddReward(reward);
                other.gameObject.SetActive(false);
                currentCheckpoint++;

                if (currentCheckpoint >= checkpoints.Length)
                {
                    currentCheckpoint = 0;
                }
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        if (steeringSlider != null && throttleSlider != null)
        {
            // Convert slider values to discrete actions
            discreteActionsOut[0] = Mathf.RoundToInt(steeringSlider.value);
            discreteActionsOut[1] = Mathf.RoundToInt(throttleSlider.value);
        }
        else
        {
            // Fallback to keyboard input if sliders are not assigned
            discreteActionsOut[0] = Mathf.RoundToInt(Input.GetAxis("Horizontal") * 30f) + 30;
            discreteActionsOut[1] = Mathf.RoundToInt(Input.GetAxis("Vertical") * 30f) + 30;
        }
    }
}