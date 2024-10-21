using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using UnityEngine.UI;

public class MultiCarAgent : Agent
{
    public float maxSteeringAngle = 30f;
    public float maxRPM = 15f;
    public float wheelRadius = 0.05f;
    public float maxMotorTorque = 300f;

    public Transform[] checkpoints;
    private int currentCheckpoint = 0;

    private float timeSinceLastCheckpoint = 0f;
    private bool isLastCheckpointCrossed = false;
    private float stopReward = 200f;

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

        // Set the slider range to match our continuous action space
        if (steeringSlider != null)
        {
            steeringSlider.minValue = -1f;
            steeringSlider.maxValue = 1f;
            steeringSlider.value = 0f;
        }
        if (throttleSlider != null)
        {
            throttleSlider.minValue = -1f;
            throttleSlider.maxValue = 1f;
            throttleSlider.value = 0f;
        }
    }

    public override void OnEpisodeBegin()
    {
        isLastCheckpointCrossed = false;
        timeSinceLastCheckpoint = 0f;
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

        RequestDecision();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Collect observations from RayPerception3D sensor
        // Assuming you have set up a RayPerceptionSensorComponent3D in the Unity Inspector
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get discrete actions
        float steeringAction = actions.ContinuousActions[0];
        float throttleAction = actions.ContinuousActions[1];

        // Convert continous actions to actual sttering and throttle values
        float steeringAngle = steeringAction * maxSteeringAngle;
        float throttleInput = (throttleAction + 1f) / 2f;


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

        // Apply speed reward and time penalty
        float currentSpeed = rigidBody.velocity.magnitude;
        AddReward(10*currentSpeed - 10f); // +1 * speed - 1 per time step

        if (isLastCheckpointCrossed)
        {
            timeSinceLastCheckpoint += Time.fixedDeltaTime;

            if (timeSinceLastCheckpoint > 5f)
            {
                // If speed is close to 0, reward for stopping
                if (currentSpeed < 0.1f)
                {
                    AddReward(stopReward);
                }
                EndEpisode();
            }
            else if (currentSpeed < 0.1f)
            {
                // Reward for stopping within 5 seconds
                AddReward(stopReward + (5f - timeSinceLastCheckpoint) * 20f);
                ResetCar();
            }
        }
        else
        {
            timeSinceLastCheckpoint = 0f;
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
                ResetCar();
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("checkpoint"))
        {
            if (checkpoints[currentCheckpoint].gameObject == other.gameObject)
            {
                float reward = 150f;
                if (currentCheckpoint == 1) reward = 200f;
                else if (currentCheckpoint == 2) reward = 250f;
                else if (currentCheckpoint == 3) reward = 300f;

                AddReward(reward);
                other.gameObject.SetActive(false);
                currentCheckpoint++;

                if (currentCheckpoint >= checkpoints.Length)
                {
                    isLastCheckpointCrossed = true;
                    timeSinceLastCheckpoint = 0f;
                    AddReward(1000f);
                }
                else
                {
                    isLastCheckpointCrossed = false;
                }
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        if (steeringSlider != null && throttleSlider != null)
        {
            // Convert slider values to continuous actions
            continuousActionsOut[0] = steeringSlider.value;
            continuousActionsOut[1] = throttleSlider.value; 
        }
        else
        {
            // Fallback to keyboard input if sliders are not assigned
            continuousActionsOut[0] = Input.GetAxis("Horizontal");
            continuousActionsOut[1] = Input.GetAxis("Vertical");
        }
    }
}
