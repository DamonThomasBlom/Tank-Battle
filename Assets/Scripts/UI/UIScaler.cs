using TriInspector;
using UnityEngine;

public class UIScaler : MonoBehaviour
{
    [Button]
    public void ScaleUI(float factor)
    {
        var scalables = GetComponentsInChildren<Scalable>();
        foreach(var scalable in scalables)
        {
            scalable.Scale(factor);
        }
    }
}
