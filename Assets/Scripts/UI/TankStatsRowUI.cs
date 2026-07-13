using TMPro;
using UnityEngine;

public class TankStatsRowUI : MonoBehaviour
{
    public TextMeshProUGUI Name;
    public TextMeshProUGUI Kills;
    public TextMeshProUGUI Deaths;
    public TextMeshProUGUI Damage;

    public void Setup(TankStats stats)
    {
        Name.text = stats.TankName;
        Kills.text = stats.Kills.ToString();
        Deaths.text = stats.Deaths.ToString();
        Damage.text = stats.Damage.ToString("F0");
    }
}