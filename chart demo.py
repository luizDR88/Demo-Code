import pandas as pd
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import seaborn as sns
import os
from mplsoccer.pitch import Pitch

folder_path = os.getcwd()
files = ['ball_calculated_data_tabela_final_fdt6_cropped.csv', 'player_calculated_data_tabela_final_fdt6_cropped.csv']
selected_file = 0
ball_df_path = os.path.join(folder_path, 'files', files[selected_file])
ball_df = pd.read_csv(ball_df_path)

selected_file = 1
player_df_path = os.path.join(folder_path, 'files', files[selected_file])
player_df = pd.read_csv(player_df_path)

ball_df['ball id'] = 1
ball_df.head()

player_df['player id'] = 1
player_df.head()

fig, ax = plt.subplots(figsize=(13.5, 8))
fig.set_facecolor('#c2d59d')
ax.patch.set_facecolor('#22312b')

# Setup the pitch
pitch = Pitch(pitch_color='grass',stripe_color='#c2d59d', stripe=True, line_color='white', axis=True, label=True, half=True,goal_type='box')
# pitch.draw(ax=ax)
# plt.gca().invert_yaxis()

# then setup the pitch plot markers we want to animate
marker_kwargs = {'marker': 'o', 'markeredgecolor': 'black', 'linestyle': 'None'}
ball, = ax.plot([], [], ms=6, markerfacecolor='w', zorder=3, **marker_kwargs)
# away, = ax.plot([], [], ms=10, markerfacecolor='#b94b75', **marker_kwargs)  # red/maroon
home, = ax.plot([], [], ms=10, markerfacecolor='#7f63b8', **marker_kwargs)  # purple

# animation function
def animate(i):
    """ Function to animate the data. Each frame it sets the data for the players and the ball."""
    # set the ball data with the x and y positions for the ith frame
    ball.set_data(ball_df.iloc[i, 6], ball_df.iloc[i, 7])
    # get the frame id for the ith frame
    frame = ball_df.iloc[i, -1]
    # set the player data using the frame id
#     away.set_data(df_away.loc[df_away.Frame == frame, 'x'],
#                   df_away.loc[df_away.Frame == frame, 'y'])
    home.set_data(player_df.loc[player_df.movement_counter == frame, 'player_pos_x'],
                  player_df.loc[player_df.movement_counter == frame, 'player_pos_y'])
    return ball, home

anim = animation.FuncAnimation(fig, animate, frames=len(ball_df), interval=50, blit=True)
plt.show()