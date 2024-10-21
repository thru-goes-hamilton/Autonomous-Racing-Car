using UnityEngine;
using Unity.MLAgents;

public class TrainingCoordinator : MonoBehaviour
{
    public MultiCarAgent[] multiCarAgents;
    public float trainingDuration = 3600f; // 1 hour
    private float trainingStartTime;

    void Start()
    {
        trainingStartTime = Time.time;
        foreach (var car in multiCarAgents)
        {
            car.OnEpisodeBegin();
        }
    }

    void Update()
    {
        if (Time.time - trainingStartTime > trainingDuration)
        {
            EndTraining();
        }
    }

    void EndTraining()
    {
        foreach (var car in multiCarAgents)
        {
            car.enabled = false;
        }
        Debug.Log("Training session ended");
    }
}