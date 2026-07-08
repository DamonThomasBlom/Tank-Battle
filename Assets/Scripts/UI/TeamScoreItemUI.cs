using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class TeamScoreItemUI : MonoBehaviour
{
    public Slider ScoreSlider;
    public TextMeshProUGUI ScoreText;
    public ColourSetter ColourSetter;

    public void Setup(int score, int maxScore, Color teamColor)
    {
        ScoreSlider.maxValue = maxScore;
        ScoreSlider.value = score;
        ScoreText.text = score.ToString();
        ColourSetter.SetColour(teamColor);
    }

    public void UpdateScore(int score)
    {
        ScoreSlider.value = score;
        ScoreText.text = score.ToString();
    }
}
