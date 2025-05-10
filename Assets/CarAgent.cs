using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public class CarAgent : Agent
{
    [Header("Agent Parameters")]
    [SerializeField] private GameObject parkingSpot;
    [SerializeField] private float moveSpeed = 10f;
    [SerializeField] private float rotateSpeed = 100f;
    [SerializeField] private float parkingDistanceThreshold = 1.0f;
    [SerializeField] private float parkingAngleThreshold = 5.0f;
    [SerializeField] private float spotCheckInterval = 0.1f;

    private Rigidbody rBody;
    private float lastSpotCheckTime;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
        rBody.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        lastSpotCheckTime = Time.time;
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(UnityEngine.Random.Range(-8f, 8f), 0.2f, UnityEngine.Random.Range(-8f, 8f));
        transform.localRotation = Quaternion.Euler(0, UnityEngine.Random.Range(0f, 360f), 0);

        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;

        lastSpotCheckTime = Time.time;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 positionRelativeToSpot = parkingSpot.transform.localPosition - transform.localPosition;
        sensor.AddObservation(positionRelativeToSpot.normalized);
        sensor.AddObservation(positionRelativeToSpot.magnitude);

        Vector3 carForward = transform.forward;
        Vector3 spotForward = parkingSpot.transform.forward;
        sensor.AddObservation(Vector3.Dot(carForward, spotForward));
        sensor.AddObservation(Vector3.Dot(transform.right, spotForward));

        sensor.AddObservation(rBody.velocity.normalized);
        sensor.AddObservation(rBody.velocity.magnitude);

        sensor.AddObservation(rBody.angularVelocity.normalized);
        sensor.AddObservation(rBody.angularVelocity.magnitude);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steerAction = actions.ContinuousActions[0];
        float accelerateAction = actions.ContinuousActions[1];
        float brakeAction = actions.ContinuousActions[2];

        float steerTorque = steerAction * rotateSpeed;
        rBody.AddTorque(transform.up * steerTorque);

        float forwardForce = accelerateAction * moveSpeed;
        rBody.AddForce(transform.forward * forwardForce);

        float brakeForce = brakeAction * moveSpeed;
        if (rBody.velocity.magnitude > 0.1f)
        {
            rBody.AddForce(-rBody.velocity.normalized * brakeForce);
        }

        AddReward(-1f / MaxStep);

        if (Time.time - lastSpotCheckTime > spotCheckInterval)
        {
            CheckParkingSuccess();
            lastSpotCheckTime = Time.time;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collided with: " + collision.gameObject.name);

        if (collision.gameObject.CompareTag("Wall"))
        {
            AddReward(-1.0f);
            EndEpisode();
        }

        if (collision.gameObject.CompareTag("ParkingSpot"))
        {
            AddReward(-1.5f);
            EndEpisode();
        }

        if (collision.gameObject.CompareTag("Ground"))
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }

    void CheckParkingSuccess()
    {
        float distanceToSpot = Vector3.Distance(transform.localPosition, parkingSpot.transform.localPosition);
        float angleDifference = Quaternion.Angle(transform.localRotation, parkingSpot.transform.localRotation);

        bool withinBounds = false;
        Collider parkingSpotCollider = parkingSpot.GetComponent<Collider>();
        if (parkingSpotCollider != null)
        {
            if (parkingSpotCollider.bounds.Contains(transform.position))
            {
                withinBounds = true;
            }
        }

        bool isStopped = rBody.velocity.magnitude < 0.1f && rBody.angularVelocity.magnitude < 0.1f;

        bool parkedSuccessfully =
            distanceToSpot <= parkingDistanceThreshold &&
            angleDifference <= parkingAngleThreshold &&
            isStopped &&
            withinBounds;

        if (parkedSuccessfully)
        {
            AddReward(5.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetKey(KeyCode.W) ? 1f : 0f;
        continuousActionsOut[2] = Input.GetKey(KeyCode.S) ? 1f : 0f;
    }

    void OnDrawGizmosSelected()
    {
        if (parkingSpot != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(parkingSpot.transform.localPosition, parkingDistanceThreshold);
        }
    }
}
