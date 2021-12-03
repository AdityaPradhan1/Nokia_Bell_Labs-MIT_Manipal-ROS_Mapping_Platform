using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class PlayerNavMesh : MonoBehaviour{

    [SerializeField] private Transform movePositionTransform;
    private NavMeshAgent navMeshAgent;
   private void Awake(){
       navMeshAgent=GetComponent<NavMeshAgent>();

   }
   private void Update(){
       if (Input.GetKeyDown(KeyCode.Space)){
           navMeshAgent.destination = movePositionTransform.position;
       }
   }
}
