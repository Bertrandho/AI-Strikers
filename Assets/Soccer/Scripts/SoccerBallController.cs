using UnityEngine;

public class SoccerBallController : MonoBehaviour
{
    public GameObject area;
    [HideInInspector]
    public SoccerEnvController envController;
    public string purpleGoalTag; //will be used to check if collided with purple goal
    public string blueGoalTag; //will be used to check if collided with blue goal

    //Add reference to the last player who touched the ball
    public AgentSoccer lastTouchedBy;
    public bool possessionLost; // Flag to indicate loss of possession 

    public SoccerBallController soccerBallController;

    void Start()
    {
        envController = area.GetComponent<SoccerEnvController>();
    }

    public void TouchedBy(AgentSoccer player)
    {
        // Check if the last player who touched the ball is not from the same team
        if (lastTouchedBy != null && lastTouchedBy.team != player.team)
        {
            possessionLost = true;
        }

        lastTouchedBy = player;
    }

    void OnCollisionEnter(Collision col)
    {
        if (col.gameObject.CompareTag(purpleGoalTag)) //ball touched purple goal
        {
            envController.GoalTouched(Team.Blue);
        }
        else if (col.gameObject.CompareTag(blueGoalTag)) //ball touched blue goal
        {
            envController.GoalTouched(Team.Purple);
        }

    }

public bool InterceptedByOpponent()
{
    if (lastTouchedBy == null) {
        // Early return or handle the case where lastTouchedBy hasn't been set yet
        return false;
    }

    // Ensure there are actually opponents to check against
    var opponents = FindObjectsOfType<AgentSoccer>();
    if (opponents == null || opponents.Length == 0) {
        return false;
    }

    foreach (var opponent in opponents)
    {
        if (opponent.team != lastTouchedBy.team && Vector3.Distance(opponent.transform.position, transform.position) < 1.0f)
        {
            return true;
        }
    }
    return false;
}

}
