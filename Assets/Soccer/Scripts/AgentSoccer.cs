using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using System;

public enum Team
{
    Blue = 0,
    Purple = 1
}

public class AgentSoccer : Agent
{
    // Note that that the detectable tags are different for the blue and purple teams. The order is
    // * ball
    // * own goal
    // * opposing goal
    // * wall
    // * own teammate
    // * opposing player

    public enum Position
    {
        Striker,
        Goalie,
        Generic
    }

    [HideInInspector]
    public Team team;
    float m_KickPower;
    // The coefficient for the reward for colliding with a ball. Set using curriculum.
    float m_BallTouch;
    public Position position;

    const float k_Power = 2000f;
    float m_Existential;
    float m_LateralSpeed;
    float m_ForwardSpeed;


    [HideInInspector]
    public Rigidbody agentRb;
    SoccerSettings m_SoccerSettings;
    BehaviorParameters m_BehaviorParameters;
    public Vector3 initialPos;
    public float rotSign;

    EnvironmentParameters m_ResetParams;


    private bool isDribbling = false;
    private float dribbleStartTime = 0f;
    private float lastDribbleTime = 0f;
    private const float dribbleGracePeriod = 0.1f; // 0.1 seconds to regain control
    private const float dribbleDurationThreshold = 0.16f; // Required time to consider successful dribbling



    public SoccerBallController soccerBallController;
    public SoccerEnvController envController;

    // Add references to the goals
    public Transform ownGoal;
    // Reference to the opposing goal's Transform
    public Transform opposingGoal;
    // Defensive threshold based on field length
    private float oneThirdOfField;

    public AgentSoccer lastTouchedBy; // Last agent to touch or kick the ball
    public bool isShotOnGoal; // Whether the last kick was a shot on goal

    private float lastDefensiveRewardTime;
    private float defensiveRewardCooldown = 3.0f; // seconds

    private float lastClusteringCheckTime;
    private float clusteringCheckCooldown = 3.0f; // seconds


    public override void Initialize()
    {
        envController = GetComponentInParent<SoccerEnvController>();
        if (envController != null)
        {
            m_Existential = 1f / envController.MaxEnvironmentSteps;
        }
        else
        {
            m_Existential = 1f / MaxStep;
        }

        m_BehaviorParameters = gameObject.GetComponent<BehaviorParameters>();
        if (m_BehaviorParameters.TeamId == (int)Team.Blue)
        {
            team = Team.Blue;
            initialPos = new Vector3(transform.position.x - 5f, .5f, transform.position.z);
            rotSign = 1f;
        }
        else
        {
            team = Team.Purple;
            initialPos = new Vector3(transform.position.x + 5f, .5f, transform.position.z);
            rotSign = -1f;
        }
        if (position == Position.Goalie)
        {
            m_LateralSpeed = 1.0f;
            m_ForwardSpeed = 1.0f;
        }
        else if (position == Position.Striker)
        {
            m_LateralSpeed = 0.3f;
            m_ForwardSpeed = 1.3f;
        }
        else
        {
            m_LateralSpeed = 0.3f;
            m_ForwardSpeed = 1.0f;
        }
        m_SoccerSettings = FindObjectOfType<SoccerSettings>();
        agentRb = GetComponent<Rigidbody>();
        agentRb.maxAngularVelocity = 500;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        // Assuming goals are tagged appropriately in your game
        ownGoal = GameObject.FindGameObjectWithTag(team == Team.Blue ? "blueGoal" : "purpleGoal").transform;
        opposingGoal = GameObject.FindGameObjectWithTag(team == Team.Blue ? "purpleGoal" : "blueGoal").transform;

        // Calculate the one third the distance between goals
        oneThirdOfField = Vector3.Distance(ownGoal.position, opposingGoal.position) / 3;
    }

    public void InitializeSoccerBallController(SoccerBallController controller)
    {
        soccerBallController = controller;
        if (soccerBallController == null)
        {
            //Debug.LogError("Failed to initialize SoccerBallController in " + gameObject.name);
        }
    }

    public void MoveAgent(ActionSegment<int> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        m_KickPower = 0f;

        var forwardAxis = act[0];
        var rightAxis = act[1];
        var rotateAxis = act[2];

        switch (forwardAxis)
        {
            case 1:
                dirToGo = transform.forward * m_ForwardSpeed;
                m_KickPower = 0.5f;
                break;
            case 2:
                dirToGo = transform.forward * -m_ForwardSpeed;
                break;
        }

        switch (rightAxis)
        {
            case 1:
                dirToGo = transform.right * m_LateralSpeed;
                break;
            case 2:
                dirToGo = transform.right * -m_LateralSpeed;
                break;
        }

        switch (rotateAxis)
        {
            case 1:
                rotateDir = transform.up * -1f;
                break;
            case 2:
                rotateDir = transform.up * 1f;
                break;
        }

        transform.Rotate(rotateDir, Time.deltaTime * 100f);
        agentRb.AddForce(dirToGo * m_SoccerSettings.agentRunSpeed,
            ForceMode.VelocityChange);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)

    {

        if (position == Position.Goalie)
        {
            // Existential bonus for Goalies.
            AddReward(m_Existential);
        }
        else if (position == Position.Striker)
        {
            // Existential penalty for Strikers
            AddReward(-m_Existential);
        }
        MoveAgent(actionBuffers.DiscreteActions);


        // Only check dribbling if there's significant movement
        if (actionBuffers.DiscreteActions[0] != 0 || actionBuffers.DiscreteActions[1] != 0)
        {
            CheckDribbling();
        }

        if (Time.time - lastClusteringCheckTime > clusteringCheckCooldown)
        {
            DiscourageClustering();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        //forward
        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
        }
        if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[0] = 2;
        }
        //rotate
        if (Input.GetKey(KeyCode.Q))
        {
            discreteActionsOut[2] = 1;
        }
        if (Input.GetKey(KeyCode.E))
        {
            discreteActionsOut[2] = 2;
        }
        //right
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[1] = 1;
        }
        if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[1] = 2;
        }
    }


    /// <summary>
    /// Used to provide a "kick" to the ball.
    /// </summary>
    void OnCollisionEnter(Collision c)
    {
        if (c.gameObject.CompareTag("ball"))
        {
            SoccerBallController ballController = c.gameObject.GetComponent<SoccerBallController>();


            // Reward or penalize for ball touches
            if (ballController.possessionLost)
            {
                // This means the ball was intercepted by an opponent after this agent touched it
                AddReward(-0.03f);  // Penalty for losing the ball
                                    //Debug.Log($"{this.name} penalized for losing possession");
                ballController.possessionLost = false; // Reset the interception flag
            }


            // Check for successful pass
            if (ballController.lastTouchedBy != null && ballController.lastTouchedBy.team == this.team && ballController.lastTouchedBy != this)
            {
                float passDistance = Vector3.Distance(ballController.lastTouchedBy.transform.position, this.transform.position);

                if (passDistance >= oneThirdOfField)
                {
                    //Debug.Log("Successful pass from " + ballController.lastTouchedBy.name + " to " + this.name);

                    // Successful pass between teammates
                    envController.RewardTeamPass(team);
                }
            }

            if (ballController.InterceptedByOpponent())
            {
                if (isDribbling)
                {
                    // Penalty for losing the ball while dribbling
                    AddReward(-0.1f);
                    //Debug.Log("Dribbling interrupted by interception for " + gameObject.name);
                    isDribbling = false;

                }
            }

            float distanceFromOwnGoal = Vector3.Distance(transform.position, ownGoal.position);

            if (distanceFromOwnGoal < oneThirdOfField)
            {
                if (team != ballController.lastTouchedBy.team) // Check if the collider is from the defending team
                {
                    Vector3 impactPoint = c.contacts[0].point;
                    Vector3 kickDirection = impactPoint - transform.position;
                    kickDirection = kickDirection.normalized;

                    // Calculate the direction towards the defending team's goal
                    Vector3 goalDirection = (opposingGoal.position - transform.position).normalized;

                    // Check if the kick direction significantly deviates from the goal direction
                    if (Vector3.Angle(kickDirection, goalDirection) > 90.0f) // More than 90 degrees means the ball is kicked away
                    {
                        DefendGoal();
                    }
                }
            }

            // Update last touched player
            ballController.TouchedBy(this);
            //Debug.Log(this.name + " touched the ball.");

            var force = k_Power * m_KickPower;
            if (position == Position.Goalie)
            {
                force = k_Power;
            }

            var dir = c.contacts[0].point - transform.position;
            dir = dir.normalized;


            if (distanceFromOwnGoal > oneThirdOfField * 2)
            {
                // Determine if the action is a shot on goal
                if (IsShotOnGoal(dir, opposingGoal))
                {
                    AddReward(0.5f); // Reward for shooting towards goal
                    //Debug.Log($"{gameObject.name} made a shot on goal!");
                }
            }

            c.gameObject.GetComponent<Rigidbody>().AddForce(dir * force);

            AddReward(.2f * m_BallTouch);
        }
    }

    public override void OnEpisodeBegin()
    {
        m_BallTouch = m_ResetParams.GetWithDefault("ball_touch", 0);
    }

    private void CheckDribbling()
    {
        float currentDistance = Vector3.Distance(transform.position, soccerBallController.transform.position);

        if (currentDistance < 1.0f)
        {
            if (!isDribbling)
            {
                isDribbling = true;
                dribbleStartTime = Time.time;
                //Debug.Log("Dribbling started by " + gameObject.name);
            }

            // Update last dribble time every frame the player is within control distance
            lastDribbleTime = Time.time;

            if (Time.time - dribbleStartTime > dribbleDurationThreshold && !soccerBallController.InterceptedByOpponent())
            {
                AddReward(0.5f);
                isDribbling = false;
                //Debug.Log("Successful dribble by " + gameObject.name);
            }
        }
        else if (isDribbling && Time.time - lastDribbleTime <= dribbleGracePeriod)
        {
            // Check if current time is within the grace period after losing proximity
            //Debug.Log("Grace period active for " + gameObject.name + ", trying to regain control.");
            // Do not reset dribbling yet, allowing a chance to regain control
        }
        else
        {
            if (isDribbling) // Only log and reset if dribbling was previously active
            {
                //Debug.Log("Dribbling reset for " + gameObject.name);
                isDribbling = false;
            }
        }
    }

    private void CheckDefensivePositioning()
    {

        if (Time.time - lastDefensiveRewardTime < defensiveRewardCooldown)
            return;

        Vector3 ballPosition = soccerBallController.transform.position;
        float ballDistanceToOwnGoal = Vector3.Distance(ballPosition, ownGoal.position);
        float agentDistanceToBall = Vector3.Distance(transform.position, ballPosition);

        //Debug.Log($"{name} Ball Position: {ballPosition}, Ball distance to Own Goal: {ballDistanceToOwnGoal}, Defensive Threshold: {oneThirdOfField}");

        // Activate defensive behavior only if the ball is within one third of the field length from the own goal
        if (ballDistanceToOwnGoal <= oneThirdOfField)
        {
            Vector3 toGoal = ownGoal.position - transform.position;
            Vector3 toBall = ballPosition - transform.position;

            //Debug.Log($"Agent {name} is checking defensive positioning. Angle to Goal: {Vector3.Angle(toGoal, toBall)}, Distance to Ball: {toBall.magnitude}");

            // Check if the angle between the agent to the goal and the agent to the ball is small
            if (Vector3.Angle(toGoal, toBall) < 45.0f)
            {
                float agentDistanceToOwnGoal = toGoal.magnitude;

                // Check if the agent is closer to the goal than to the ball
                if (agentDistanceToOwnGoal < agentDistanceToBall)
                {
                    float reward = 0.5f * (1.0f - Vector3.Angle(toGoal, toBall) / 45.0f); // Normalize reward based on angle
                    AddReward(reward);
                    //Debug.Log($"Agent {name} rewarded for defensive positioning: {reward}");
                    lastDefensiveRewardTime = Time.time; // Update last reward time
                }
            }
        }
    }

    private bool IsShotOnGoal(Vector3 kickDirection, Transform goal)
    {
        Vector3 toGoal = goal.position - transform.position;
        float angle = Vector3.Angle(kickDirection, toGoal);

        // Consider it a shot on goal if the angle is within a certain threshold, e.g., 30 degrees
        return angle <= 30.0f;
    }

    public void DefendGoal()
    {
        // Reward the agent for defending the goal
        AddReward(0.8f);  // Adjust reward magnitude based on your reward scheme
        //Debug.Log("Goal defended successfully by " + gameObject.name);
    }

    private void DiscourageClustering()
    {
        foreach (var otherAgent in FindObjectsOfType<AgentSoccer>())
        {
            if (otherAgent != this && otherAgent.team == this.team)
            {
                float distance = Vector3.Distance(transform.position, otherAgent.transform.position);
                if (distance < 5.5f) // Example threshold for 'too close'
                {
                    AddReward(-0.02f); // Penalize for being too close
                    //Debug.Log("too close");
                }
            }
        }
    }

}
