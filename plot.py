import matplotlib.pyplot as plt
import matplotlib.collections as collections
import pandas as pd


tuples = [
    ('nis-lidar', 2, 5.991),
    ('nis-radar', 3, 7.815),
]

for filename, dof, threshold_005 in tuples:

    df = pd.read_csv(f'{filename}.csv', skipinitialspace=True)
    grouped = df.groupby('car')


    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color']

    fig, ax = plt.subplots(len(grouped), 1, figsize=(8, 10))
    fig.suptitle(f'Normalized Innovation Squared ($\chi^2$ at {dof} DOF)', fontsize=12)

    for i, (key, group) in enumerate(grouped):
        t = group['time [s]'].values
        x = group['NIS'].values

        # Percentage of values below threshold
        count = group[group['NIS'] < threshold_005]['NIS'].count()
        p = count / len(x)

        ax[i].plot([min(t), max(t)], [threshold_005, threshold_005], ':', color='black', alpha=0.5)
        group.plot(ax=ax[i], kind='line', x='time [s]', y='NIS', label=f'car {key}', color=colors[i])
        ax[i].set_ylabel('NIS')
        ax[i].set_title(f'p(NIS < {threshold_005}) = {p:.2}')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(f'{filename}.png')
