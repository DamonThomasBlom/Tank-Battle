using UnityEngine;

public class Scalable : MonoBehaviour
{
    public void Scale(float factor)
    {
        transform.localScale = new Vector3(factor, factor, factor);
    }
}
