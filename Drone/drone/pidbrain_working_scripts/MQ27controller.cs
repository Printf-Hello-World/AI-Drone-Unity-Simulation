using UnityEngine;
using System.Collections;
using MLAgents;
public class MQ27controller : Agent
{
    //The propellers
    public GameObject propellerFR;
    public GameObject propellerFL;
    public GameObject propellerBL;
    public GameObject propellerBR;


    private float propellerForceFR;
    private float propellerForceFL;
    private float propellerForceBL;
    private float propellerForceBR;

    //Quadcopter parameters
    [Header("Internal")]
    public float maxPropellerForce;
    public float maxTorque;
    public float throttle;
    public float moveFactor;
    //PID
    public Vector3 PID_pitch_gains;
    public Vector3 PID_roll_gains;
    public Vector3 PID_yaw_gains;

    //External parameters
    [Header("External")]
    public float windForce;
    //0 -> 360
    public float forceDir;

    Rigidbody quadcopterRB;
    //The PID controllers
    private PIDController PID_pitch;
    private PIDController PID_roll;
    private PIDController PID_yaw;

    //Agent variables
    public GameObject target;
    
    

    [SerializeField]
    private Vector3 directiontotarget;
    [SerializeField]
    private Vector3 directiontotarget_normalized;
    [SerializeField]
    private float currentdistance;
    [SerializeField]
    private float originaldistance;
    [SerializeField]
    private Vector3 targetposition;


    [SerializeField]
    private float pitch;
    [SerializeField]
    private float yaw;
    [SerializeField]
    private float roll;

    private float anglediff;
    private float previousdistance;
    private float improvedistancereward;
    private float totalreward;
    private float energyused;
    private float stable;



    //Movement factors
    public float moveForwardBack;
    public float moveLeftRight;
    public float yawDir;
    

    void Start()
    {
        quadcopterRB = gameObject.GetComponent<Rigidbody>();
        PID_pitch = new PIDController();
        PID_roll = new PIDController();
        PID_yaw = new PIDController();
        Monitor.SetActive(true);
    }


    public override void AgentReset()
    {
        target.transform.localPosition = new Vector3(Random.Range(-3f, 3f), Random.Range(-1f, 5f), Random.Range(1f, 5f));
        //target.transform.localPosition = new Vector3(0f, Random.Range(-1f, 6f),0f);
        this.transform.localPosition = Vector3.zero;
        this.transform.localRotation = Quaternion.identity;
        quadcopterRB.velocity = Vector3.zero;
        quadcopterRB.angularVelocity = Vector3.zero;
        throttle = 0;
        originaldistance = Vector3.Distance(this.transform.position, target.transform.position);
        previousdistance = Vector3.Distance(this.transform.position, target.transform.position);


    }

    public override void AgentAction(float[] vectorAction) //4 branches 2 actions each branch
    {
        //index for throttle
        int throttleaction = Mathf.FloorToInt(vectorAction[0]);
        //index for pitch
        int pitchaction = Mathf.FloorToInt(vectorAction[1]);
        //index for roll
        int rollaction = Mathf.FloorToInt(vectorAction[2]);
        //index for yaw
        int yawaction = Mathf.FloorToInt(vectorAction[3]);

        //throttle

        if(throttleaction == 1) { throttle += 5f; }
        else if(throttleaction == 2) { throttle -= 5f; }
        throttle = Mathf.Clamp(throttle, 0f, 200);

        //Steering
        //Move forward or reverse
        moveForwardBack = 0f;
        if (pitchaction == 1) { moveForwardBack = 1f; }
        else if(pitchaction == 2) { moveForwardBack = -1f; }

        //Move left or right
        moveLeftRight = 0f;
        if (rollaction == 1) { moveLeftRight = 1f; }
        else if (rollaction == 2) { moveLeftRight = -1f; }

        //Rotate around the axis
        yawDir = 0f;
        if (yawaction == 1) { yawDir = 1f; }
        else if (yawaction == 2) { yawDir = -1f; }
        energyused = AddMotorForce();
        AddExternalForces();
        AddReward(rewards());
        Debug.Log(rewards());
    }

    private void OnTriggerEnter(Collider other)
    {
        if(other.CompareTag("target"))
        {
            AddReward(1.0f);
            Done();
        }
    }

    public override void CollectObservations()
    {
        if(throttle == 0)
        {
            SetActionMask(1, new int[2] { 1, 2 });
            SetActionMask(2, new int[2] { 1, 2 });
            SetActionMask(3, new int[2] { 1, 2 });
        }

        Vector3 pitchplane;
        Vector3 yawplane;
        Vector3 rollplane;

        rollplane = Vector3.ProjectOnPlane(transform.right, new Vector3(0f,1f,0f));
        yawplane = Vector3.ProjectOnPlane(transform.forward, new Vector3(1f, 0f, 0f));
        pitchplane = Vector3.ProjectOnPlane(transform.forward, new Vector3(0f, 1f, 0f));

        pitch = Vector3.SignedAngle(transform.forward, pitchplane, transform.right);
        roll = Vector3.SignedAngle(transform.right, rollplane, transform.forward);
        yaw = Vector3.SignedAngle(transform.forward, yawplane, transform.up);


        //Debug.DrawRay(transform.position, new Vector3(0f, transform.forward.y, transform.forward.z)*100f);
        //Debug.DrawRay(transform.position, pitchplane * 100f, Color.green);
        //Debug.DrawRay(transform.position, yawplane * 100f, Color.red);
        //Debug.DrawRay(transform.position, rollplane * 100f, Color.black);

        directiontotarget = target.transform.localPosition - this.transform.localPosition;
        directiontotarget_normalized = Vector3.Normalize(directiontotarget);
        currentdistance = Vector3.Distance(this.transform.position, target.transform.position);

        AddVectorObs(throttle); //1
        AddVectorObs(pitch); //1
        AddVectorObs(yaw); //1
        AddVectorObs(roll); //1
        AddVectorObs(currentdistance); //1
        AddVectorObs(directiontotarget_normalized); //3
        AddVectorObs(transform.localPosition); //3
        AddVectorObs(quadcopterRB.velocity.x); //1
        AddVectorObs(quadcopterRB.velocity.z);//1
        AddVectorObs(quadcopterRB.velocity.y);//1


        Debug.DrawLine(transform.position, target.transform.position);
        Debug.DrawRay(this.transform.position, this.transform.forward * 100);
        Debug.DrawRay(this.transform.position, this.transform.right * 100);
        


        //14 observations
    }

    public float rewards()
    {
        float totalreward = 0f;
        Vector3 heading;
        Vector3 correctheading;
        float headingdiff;
        float energyaward;
        currentdistance = Vector3.Distance( this.transform.localPosition, target.transform.localPosition);
        stable = Vector3.Angle(this.transform.up, target.transform.up)/180f;
        improvedistancereward = (previousdistance - currentdistance)/previousdistance;
        heading = Vector3.ProjectOnPlane(this.transform.forward, new Vector3(0f, 1f, 0f));
        correctheading = Vector3.ProjectOnPlane(directiontotarget, new Vector3(0f, 1f, 0f));
        headingdiff = Vector3.Angle(new Vector3(heading.x, 0f, heading.z), new Vector3(correctheading.x, 0f, correctheading.z))/180f;
        energyaward = energyused * Mathf.Abs(improvedistancereward);
        totalreward = improvedistancereward*100f - headingdiff*10f - energyaward*100f - stable;


        Debug.DrawRay(this.transform.position, heading);
        Debug.DrawRay(this.transform.position, this.transform.up);
        Debug.DrawRay(this.transform.position, this.transform.forward);
        previousdistance = currentdistance;

        return totalreward;
    }
    

    public override float[] Heuristic()
    {
        var action = new float[4];
        if (Input.GetKey("up"))
        {
            action[0] = 1f;
        }
        if (Input.GetKey("down"))
        {
            action[0] = 2f;
        }
        if (Input.GetKey("w"))
        {
            action[1] = 1f;
        }
        if (Input.GetKey("s"))
        {
            action[1] = 2f;
        }
        if (Input.GetKey("d"))
        {
            action[2] = 1f;
        }
        if (Input.GetKey("a"))
        {
            action[2] = 2f;
        }
        if (Input.GetKey("right"))
        {
            action[3] = 1f;
        }
        if (Input.GetKey("left"))
        {
            action[3] = 2f;
        }
        return action;
    }


    float AddMotorForce()
    {
        //Calculate the errors so we can use a PID controller to stabilize
        //Assume no error is if 0 degrees

        //Pitch
        //Returns positive if pitching forward
        float pitchError = GetPitchError();

        //Roll
        //Returns positive if rolling left
        float rollError = GetRollError() * -1f;

        //Adapt the PID variables to the throttle
        Vector3 PID_pitch_gains_adapted = throttle > 100f ? PID_pitch_gains * 2f : PID_pitch_gains;

        //Get the output from the PID controllers
        float PID_pitch_output = PID_pitch.GetFactorFromPIDController(PID_pitch_gains_adapted, pitchError);
        Debug.Log(PID_pitch_gains_adapted);
        float PID_roll_output = PID_roll.GetFactorFromPIDController(PID_roll_gains, rollError);

        //Calculate the propeller forces
        //FR
        float propellerForceFR = throttle + (PID_pitch_output + PID_roll_output);

        //Add steering

        propellerForceFR -= moveForwardBack * throttle * moveFactor;
        propellerForceFR -= moveLeftRight * throttle;


        //FL
        float propellerForceFL = throttle + (PID_pitch_output - PID_roll_output);

        propellerForceFL -= moveForwardBack * throttle * moveFactor;
        propellerForceFL += moveLeftRight * throttle;


        //BR
        float propellerForceBR = throttle + (-PID_pitch_output + PID_roll_output);

        propellerForceBR += moveForwardBack * throttle * moveFactor;
        propellerForceBR -= moveLeftRight * throttle;


        //BL 
        float propellerForceBL = throttle + (-PID_pitch_output - PID_roll_output);

        propellerForceBL += moveForwardBack * throttle * moveFactor;
        propellerForceBL += moveLeftRight * throttle;


        //Clamp
        propellerForceFR = Mathf.Clamp(propellerForceFR, 0f, maxPropellerForce);
        propellerForceFL = Mathf.Clamp(propellerForceFL, 0f, maxPropellerForce);
        propellerForceBR = Mathf.Clamp(propellerForceBR, 0f, maxPropellerForce);
        propellerForceBL = Mathf.Clamp(propellerForceBL, 0f, maxPropellerForce);

        //Add the force to the propellers
        AddForceToPropeller(propellerFR, propellerForceFR);
        AddForceToPropeller(propellerFL, propellerForceFL);
        AddForceToPropeller(propellerBR, propellerForceBR);
        AddForceToPropeller(propellerBL, propellerForceBL);

        //Yaw
        //Minimize the yaw error (which is already signed):
        float yawError = quadcopterRB.angularVelocity.y;

        float PID_yaw_output = PID_yaw.GetFactorFromPIDController(PID_yaw_gains, yawError);

        //First we need to add a force (if any)
        quadcopterRB.AddTorque(transform.up * yawDir * maxTorque * throttle);

        //Then we need to minimize the error
        quadcopterRB.AddTorque(transform.up * throttle * PID_yaw_output * -1f);

        return (propellerForceBL + propellerForceBR + propellerForceFL + propellerForceFR)/(4*maxPropellerForce);
    }

    public void AddForceToPropeller(GameObject propellerObj, float propellerForce)
    {
        Vector3 propellerUp = propellerObj.transform.up;

        Vector3 propellerPos = propellerObj.transform.position;

        quadcopterRB.AddForceAtPosition(propellerUp * propellerForce, propellerPos);

        //Debug
        Debug.DrawRay(propellerPos, propellerUp * 1f, Color.red);
    }

    //Pitch is rotation around x-axis
    //Returns positive if pitching forward
    private float GetPitchError()
    {
        float xAngle = transform.eulerAngles.x;

        //Make sure the angle is between 0 and 360
        xAngle = WrapAngle(xAngle);

        //This angle going from 0 -> 360 when pitching forward
        //So if angle is > 180 then it should move from 0 to 180 if pitching back
        if (xAngle > 180f && xAngle < 360f)
        {
            xAngle = 360f - xAngle;

            //-1 so we know if we are pitching back or forward
            xAngle *= -1f;
        }

        return xAngle;
    }

    //Roll is rotation around z-axis
    //Returns positive if rolling left
    private float GetRollError()
    {
        float zAngle = transform.eulerAngles.z;

        //Make sure the angle is between 0 and 360
        zAngle = WrapAngle(zAngle);

        //This angle going from 0-> 360 when rolling left
        //So if angle is > 180 then it should move from 0 to 180 if rolling right
        if (zAngle > 180f && zAngle < 360f)
        {
            zAngle = 360f - zAngle;

            //-1 so we know if we are rolling left or right
            zAngle *= -1f;
        }

        return zAngle;
    }

    //Wrap between 0 and 360 degrees
    float WrapAngle(float inputAngle)
    {
        //The inner % 360 restricts everything to +/- 360
        //+360 moves negative values to the positive range, and positive ones to > 360
        //the final % 360 caps everything to 0...360
        return ((inputAngle % 360f) + 360f) % 360f;
    }

    //Add external forces to the quadcopter, such as wind
    private void AddExternalForces()
    {
        //Important to not use the quadcopters forward
        Vector3 windDir = -Vector3.forward;

        //Rotate it 
        windDir = Quaternion.Euler(0, forceDir, 0) * windDir;

        quadcopterRB.AddForce(windDir * windForce);

        //Debug
        //Is showing in which direction the wind is coming from
        //center of quadcopter is where it ends and is blowing in the direction of the line
        //Debug.DrawRay(transform.position, -windDir * 3f, Color.red);
    }
}
