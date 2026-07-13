using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ScoreboardUI : MonoBehaviour
{
    public List<Transform> TeamColumns;
    public TankStatsRowUI RowPrefab;

    readonly List<TankStatsRowUI> rows = new();

    private void OnEnable()
    {
        InvokeRepeating(nameof(Rebuild), 1f, 0.5f);
    }

    private void OnDisable()
    {
        CancelInvoke();
    }

    void Rebuild()
    {
        while (rows.Count < TankStats.All.Count)
        {
            rows.Add(Instantiate(RowPrefab));
        }

        foreach (var r in rows)
            r.gameObject.SetActive(false);

        int rowIndex = 0;

        for (int team = 0; team < TeamColumns.Count; team++)
        {
            var sorted = TankStats.All
                .Where(t => t.TeamId == team)
                .OrderByDescending(t => t.Kills);

            foreach (var stats in sorted)
            {
                var row = rows[rowIndex++];

                row.transform.SetParent(TeamColumns[team], false);
                row.Setup(stats);
                row.gameObject.SetActive(true);
            }
        }
    }
}