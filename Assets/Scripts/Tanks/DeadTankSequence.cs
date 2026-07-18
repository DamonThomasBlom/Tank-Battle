using System;
using System.Collections.Generic;
using TriInspector;
using UnityEngine;

[Serializable]
public class DeadTankSequence : MonoBehaviour
{
    [Serializable]
    public class TransformReferencePair
    {
        public Transform Reference;
        public Transform Target;
    }

    public List<TransformReferencePair> TransformReferences = new();
    public Transform ReferenceRoot;
    public Transform TargetRoot;

    private void Start()
    {
        var health = GetComponentInParent<Health>();
        if (health != null)
        {
            health.OnDie.AddListener(Die);
            health.OnRespawn.AddListener(ResetDeadTank);
        }
    }

    [Button]
    public void Die()
    {
        if (ReferenceRoot == null || TargetRoot == null)
            return;

        TargetRoot.position = ReferenceRoot.position;
        TargetRoot.rotation = ReferenceRoot.rotation;
        TargetRoot.localScale = ReferenceRoot.localScale;

        foreach (var pair in TransformReferences)
        {
            if (pair.Reference == null || pair.Target == null)
                continue;

            pair.Target.position = pair.Reference.position;
            pair.Target.rotation = pair.Reference.rotation;
            pair.Target.localScale = pair.Reference.localScale;
        }

        TargetRoot.gameObject.SetActive(true);
    }

    [Button]
    public void ResetDeadTank()
    {
        TargetRoot.gameObject.SetActive(false);
    }
}
