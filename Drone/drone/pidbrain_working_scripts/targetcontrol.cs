using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class targetcontrol : MonoBehaviour
{
    // Start is called before the first frame update
    public Rigidbody rb;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        float frontback = Input.GetAxis("Horizontal");
        float leftright = Input.GetAxis("Vertical");
        float updown = Input.GetAxis("Fire3");
        rb.AddForce(transform.forward*frontback);
        rb.AddForce(transform.right * -leftright);
        rb.AddForce(transform.up * updown);
    }
}
