using UnityEngine;

public class Player : MonoBehaviour
{
    #region SINGLETON

    // The static instance of the class
    private static Player _instance;

    // Public property to access the instance
    public static Player Instance
    {
        get
        {
            // If the instance doesn't exist, create it
            if (_instance == null)
            {
                GameObject singletonObject = new GameObject("Player (Singleton)");
                _instance = singletonObject.AddComponent<Player>();
            }

            return _instance;
        }
    }

    private void Awake()
    {
        // Ensure there is only one instance
        if (_instance != null && _instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            _instance = this;
            DontDestroyOnLoad(this.gameObject);
        }
    }

    #endregion

    public string PlayerName => DebugName;
    public string DebugName = "Damon Debug";
    public int TeamID = 0;
}


public struct GameSettings
{

}

public struct InGameSettings
{
    public bool ShowPlayerNames { get; set; }
    public bool HideHud { get; set; }
    public bool ShowPing { get; set; }
    public bool ShowFPS { get; set; }
}
