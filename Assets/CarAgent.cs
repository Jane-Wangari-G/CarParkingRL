using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public class CarAgent : Agent
{
    // --- Public Variables / Serialized Fields (Set these in the Unity Inspector) ---
    [Header("Agent Parameters")]
    [SerializeField] private GameObject parkingSpot; // Drag your ParkingSpot GameObject here in the Inspector
    [SerializeField] private float moveSpeed = 10f; // Base speed for acceleration/braking
    [SerializeField] private float rotateSpeed = 100f; // Base speed for steering rotation
    [SerializeField] private float parkingDistanceThreshold = 1.0f; // How close the car needs to be to the spot center to be considered parked
    [SerializeField] private float parkingAngleThreshold = 5.0f; // How aligned the car needs to be (in degrees) to be considered parked
    [SerializeField] private float spotCheckInterval = 0.1f; // How often (in seconds) to check for parking success (performance optimization)

    // --- Private Variables ---
    private Rigidbody rBody;
    private float lastSpotCheckTime; // Timer to control frequency of parking checks

    // --- Initialization ---
    // Called once when the script starts
    void Start()
    {
        // Get the Rigidbody component attached to this GameObject
        rBody = GetComponent<Rigidbody>();

        // Freeze Y position and X/Z rotation if you only want movement on a flat plane
        // This is crucial for keeping the car stable unless you specifically want it to tip.
        rBody.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // Initialize the timer
        lastSpotCheckTime = Time.time;
    }

    // --- Episode Reset ---
    // Called at the start of each training episode (when the agent succeeds or fails)
    public override void OnEpisodeBegin()
    {
        // --- Reset Agent Position and Rotation ---
        // This is crucial for starting new attempts. Randomizing helps the agent learn to park from different situations.
        // Adjust the random ranges to fit the size and layout of your parking environment.
        transform.localPosition = new Vector3(UnityEngine.Random.Range(-8f, 8f), 0.2f, UnityEngine.Random.Range(-8f, 8f)); // Example random start within a box
        transform.localRotation = Quaternion.Euler(0, UnityEngine.Random.Range(0f, 360f), 0); // Example random rotation

        // --- Reset Agent Velocity ---
        // Stop the car completely at the start of a new episode
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;

        // --- Reset Environment Elements (if dynamic) ---
        // If you add moving obstacles or randomize the parking spot location (Grading Criterion 2, 4),
        // make sure to reset/randomize their positions/states here as well in their respective scripts or this one if they are children.

        // Reset the timer
        lastSpotCheckTime = Time.time;
    }

    // --- Collect Observations (What the Agent Sees) ---
    // This is where you gather all the data the agent will use to make decisions (Grading Criterion 5 - Using Sensors)
    // Remember the order and total count must match your Behavior Parameters > Vector Observation Space Size in the Inspector!
    // Based on the manual observations below, this method adds 14 values.
    // If you use a Ray Perception Sensor 3D component, its observations are added automatically IN ADDITION to these.
    // Ensure the total (manual + sensor components) matches the Space Size in the Inspector!
    public override void CollectObservations(VectorSensor sensor)
    {
        // --- Car's State relative to Parking Spot ---

        // 1. Relative Position (Car's position relative to Parking Spot) (3 values)
        // Provides direction and distance to the target. Normalizing helps with scale independence.
        Vector3 positionRelativeToSpot = parkingSpot.transform.localPosition - transform.localPosition;
        sensor.AddObservation(positionRelativeToSpot.normalized); // Add normalized vector (direction)
        sensor.AddObservation(positionRelativeToSpot.magnitude); // Add distance as a scalar

        // 2. Relative Orientation (Car's rotation relative to Parking Spot's rotation) (Example using forward/right vectors - 2 values)
        // Provides how well the car is aligned with the target spot's orientation.
        Vector3 carForward = transform.forward;
        Vector3 spotForward = parkingSpot.transform.forward;
        sensor.AddObservation(Vector3.Dot(carForward, spotForward)); // Dot product of forward vectors (1 at same, -1 at opposite, 0 at perpendicular)
        sensor.AddObservation(Vector3.Dot(transform.right, spotForward)); // Dot product of car's right vector with spot's forward (0 at correct alignment, 1 or -1 at perpendicular)


        // 3. Car's Velocity (3 values)
        // How fast and in what direction the car is currently moving.
        sensor.AddObservation(rBody.velocity.normalized); // Add normalized velocity (direction)
        sensor.AddObservation(rBody.velocity.magnitude); // Add speed as a scalar

        // 4. Car's Angular Velocity (3 values)
        // How fast the car is currently rotating.
        sensor.AddObservation(rBody.angularVelocity.normalized); // Add normalized angular velocity (axis of rotation)
        sensor.AddObservation(rBody.angularVelocity.magnitude); // Add angular speed as a scalar

        // Total manual observations added here: 3 + 1 + 1 + 1 + 3 + 1 + 3 + 1 = 14 values.
        // Remember to account for any Ray Perception Sensor 3D output in the Inspector's Space Size!
    }

    // --- OnActionReceived (Applying Actions and Adding Step Rewards) ---
    // This method receives the continuous action values decided by the RL algorithm (Grading Criteria 6 & 7 - Continuous & Higher Dimensional Actions)
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Read the continuous action values from the received ActionBuffers
        // The order matches the order in your Continuous Actions list in Behavior Parameters (Action 0, Action 1, Action 2...)
        // These values will typically be in the range [-1, 1] if using algorithms like PPO.
        float steerAction = actions.ContinuousActions[0]; // Action 0: Steering input (-1 for left, 1 for right)
        float accelerateAction = actions.ContinuousActions[1]; // Action 1: Acceleration input (e.g., 0 to 1 for forward)
        float brakeAction = actions.ContinuousActions[2]; // Action 2: Braking input (e.g., 0 to 1 for braking)

        // --- Apply Actions to the Car's Rigidbody ---
        // This is how the agent actually controls the car's movement and rotation.

        // Steering: Apply torque around the car's up axis
        float steerTorque = steerAction * rotateSpeed;
        rBody.AddTorque(transform.up * steerTorque);

        // Acceleration: Apply force forward
        // We map the accelerateAction [0, 1] to a forward force.
        float forwardForce = accelerateAction * moveSpeed;
        rBody.AddForce(transform.forward * forwardForce);

        // Braking: Apply force opposite to the current velocity
        // We map the brakeAction [0, 1] to a braking force.
        float brakeForce = brakeAction * moveSpeed; // Use moveSpeed magnitude for braking force magnitude
        // Apply braking force only if moving significantly, and opposite to current velocity
        if (rBody.velocity.magnitude > 0.1f) // Only brake if moving to avoid weird behavior when stopped
        {
             rBody.AddForce(-rBody.velocity.normalized * brakeForce);
        }
         // If brakeAction is 1 and accelerateAction is 0, this will apply max braking force.
         // If accelerateAction is 1 and brakeAction is 0, this will apply max forward force.
         // If both are 0, the car will coast (due to friction/drag if configured).
         // If both are 1, you need to decide which takes precedence in your logic, or let forces combine (can be unstable).
         // For simplicity, let's assume simultaneous actions add forces, but braking force opposes velocity.


        // --- Step Rewards (Given at every physics step) ---

        // 1. Time Penalty (Encourage efficient parking)
        // Subtract a small reward each step. This incentivizes the agent to reach the goal quickly.
        // MaxStep is the maximum number of steps allowed per episode (set in Behavior Parameters or YAML).
        AddReward(-1f / MaxStep); // Example: A penalty that ensures max possible reward over an episode is 0 if no success

        // --- Optional: Dense Rewards (Helps guide early learning towards the goal) ---
        // Uncommented as part of Experiment 1
        // 2. Proximity Reward (Encourage getting closer to the parking spot)
        // Calculate distance to the spot and give a small reward based on it.
        float distanceToSpot = Vector3.Distance(transform.localPosition, parkingSpot.transform.localPosition);
        // Rewards less as distance increases, max 0.01 at distance 0. Adjust max distance (e.g., 15f) if needed.
        AddReward( Mathf.Lerp(0.01f, 0f, distanceToSpot / 15f) );

        // 3. Orientation Reward (Encourage aligning with the parking spot)
        // Calculate angle difference and give a small reward based on it.
        float angleDifference = Quaternion.Angle(transform.localRotation, parkingSpot.transform.localRotation);
        // Rewards less as angle difference increases, max 0.01 at 0 degrees.
        AddReward( Mathf.Lerp(0.01f, 0f, angleDifference / 180f) );


        // --- Check for Parking Success (Less frequent check for performance) ---
        // Checking parking conditions every single physics step can be slightly inefficient.
        // We use a timer to check periodically.
        if (Time.time - lastSpotCheckTime > spotCheckInterval)
        {
             CheckParkingSuccess(); // Call the method to check parking criteria
             lastSpotCheckTime = Time.time; // Reset the timer
        }
    }

    // --- OnCollisionEnter (Detecting Collisions and Adding Negative Rewards) ---
    // This method is called automatically by Unity when the car's collider hits another collider.
    void OnCollisionEnter(Collision collision)
    {
        // Optional: Debug log to see what the car is colliding with. Can comment out to reduce console spam.
        Debug.Log("Collided with: " + collision.gameObject.name);

        // --- Penalize Collisions (Defines Failure Conditions) ---

        // Check if the collision is with an object tagged "Wall"
        if (collision.gameObject.CompareTag("Wall"))
        {
            // Give a significant negative reward for hitting a wall.
            AddReward(-1.0f); // Example negative reward (e.g., -0.1f to -1.0f). -1 is quite punitive.
            EndEpisode(); // End the episode immediately on hitting a wall. This marks a failed attempt.
        }

        // Check if the collision is with a dedicated "Obstacle" tag (If you add obstacles later - Grading Criterion 2)
        if (collision.gameObject.CompareTag("Obstacle")) // You would need to create and use the "Obstacle" tag in Unity
        {
             AddReward(-1.5f); // Maybe a higher penalty for hitting obstacles
             EndEpisode(); // End the episode on hitting an obstacle
        }

        // Check if the collision is with a boundary outside the parking area (If you define bounds)
        // RECOMMENDED: Add invisible colliders around your playable area and tag them "Boundary"
        if (collision.gameObject.CompareTag("Boundary")) // You would need to create and use the "Boundary" tag in Unity
        {
             AddReward(-1.0f); // Penalty for going out of bounds
             EndEpisode(); // End the episode
        }

        // PENALIZING COLLIDING WITH THE PARKING SPOT OBJECT ITSELF IS REMOVED
        // The agent should be rewarded for being IN the spot, not penalized for touching the spot's collider.
        // if (collision.gameObject.CompareTag("ParkingSpot"))
        // {
        //      AddReward(-1.5f);
        //      EndEpisode();
        // }

        // Check if the collision is with the Ground (Can be used as a simple boundary if Ground covers the whole area)
        // Consider using dedicated "Boundary" colliders instead for clearer intent.
        if (collision.gameObject.CompareTag("Ground"))
        {
             // Penalizing hitting the ground itself might be too harsh if the car is just driving on it.
             // This code will end the episode if it touches the ground.
             // AddReward(-1.0f); // Example penalty for hitting ground
             // EndEpisode(); // End the episode
        }
    }

    // --- Method to Check for Successful Parking ---
    // Contains the logic to determine if the agent has met the parking criteria.
    void CheckParkingSuccess()
    {
        // 1. Check Distance to Parking Spot Center
        float distanceToSpot = Vector3.Distance(transform.localPosition, parkingSpot.transform.localPosition);

        // 2. Check Orientation Alignment with Parking Spot
        float angleDifference = Quaternion.Angle(transform.localRotation, parkingSpot.transform.localRotation);

        // 3. Check if car is roughly within the bounds of the parking spot
        // This requires your ParkingSpot GameObject to have a Collider component (like a BoxCollider) that defines the parkable area.
        bool withinBounds = false; // Initialize to false
        Collider parkingSpotCollider = parkingSpot.GetComponent<Collider>();

        if (parkingSpotCollider != null)
        {
             // Simple check if the car's origin (transform.position) is within the bounding box of the spot's collider.
             // For better results, you might check if the car's entire collider is inside a trigger collider for the spot,
             // or check the position of all four wheels relative to the spot.
             if (parkingSpotCollider.bounds.Contains(transform.position))
             {
                 withinBounds = true;
             }
        }
        // Optional: Log a warning if the ParkingSpot doesn't have a collider, so you know why the bounds check might fail.
        // else { Debug.LogWarning("ParkingSpot GameObject is missing a Collider component. Bounds check will not function."); }


        // 4. Check if the car is stopped or moving very slowly
        // The agent must stop inside the spot to be considered parked.
        bool isStopped = rBody.velocity.magnitude < 0.1f && rBody.angularVelocity.magnitude < 0.1f; // Threshold for "stopped" speed and rotation


        // --- Define Success Criteria ---
        // The car is considered successfully parked if ALL conditions are met:
        // - It's within the required distance from the spot center.
        // - It's aligned within the required angle.
        // - It has come to a stop.
        // - It is located within the defined bounds of the parking spot (using the collider check).
        bool parkedSuccessfully =
             distanceToSpot <= parkingDistanceThreshold && // Is it close enough?
             angleDifference <= parkingAngleThreshold &&  // Is it aligned enough?
             isStopped &&                                // Has it stopped?
             withinBounds;                                 // Is it actually inside the spot?


        // --- Reward Success ---
        // If the car has met ALL the parking criteria, give a large positive reward and end the episode.
        if (parkedSuccessfully)
        {
             AddReward(5.0f); // **Large positive reward for successful parking!** Adjust this value - it should be significantly higher than any single penalty.
             EndEpisode(); // **End the episode immediately on successful parking.**
        }
    }


    // --- Heuristic Method (Manual Control for Testing) ---
    // Implement this to control the agent manually with keyboard input.
    // Useful for testing environment setup, car controls, and reward function logic before training.
    // To use: Set Behavior Type to 'Heuristic Only' in the Agent's Behavior Parameters in the Inspector.
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Get the output buffer for continuous actions
        var continuousActionsOut = actionsOut.ContinuousActions;

        // Map keyboard input to the continuous action values [-1, 1] or [0, 1]
        // These map to your defined actions: [0] Steering, [1] Acceleration, [2] Braking
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // Standard Unity Input Axis for A/D or Left/Right arrows (-1 to 1)
        continuousActionsOut[1] = Input.GetKey(KeyCode.W) ? 1f : 0f; // W key for Acceleration (1 if pressed, 0 otherwise)
        continuousActionsOut[2] = Input.GetKey(KeyCode.S) ? 1f : 0f; // S key for Braking (1 if pressed, 0 otherwise)

        // Note: Input.GetAxis("Vertical") gives -1 for Down/S and 1 for Up/W.
        // You could use GetAxis("Vertical") and map it differently if you prefer,
        // but separate W and S for Accel/Brake provides more distinct control signals.
    }

    // --- Optional: Gizmos for Visualization ---
    // Draws helpers in the Scene view (visible when the Agent is selected)
    void OnDrawGizmosSelected()
    {
        // Draw spheres at the parking thresholds
        if (parkingSpot != null)
        {
            Gizmos.color = Color.green;
            // Draw a sphere representing the distance threshold for parking
            Gizmos.DrawWireSphere(parkingSpot.transform.localPosition, parkingDistanceThreshold);

            // You could add more complex gizmos to visualize orientation thresholds or raycasts here
        }
    }
}