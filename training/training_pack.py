from pathlib import Path
from pathing_training import OpenNet, SideShot, ToughAngle, WallShot

import rlbottraining.common_exercises.bronze_goalie as bronze_goalie
from rlbot.matchconfig.match_config import PlayerConfig, Team

def make_default_playlist():
    exercises = (
        OpenNet("Easy open net"),
        SideShot("Harder side shot"),
        WallShot("Wall Shot!"),
        ToughAngle("Difficult angle"),
    )
    for exercise in exercises:
        exercise.match_config.player_configs = [
            PlayerConfig.bot_config(
                Path(__file__).absolute().parent.parent / 'src' / 'bot.cfg',
                Team.BLUE
        ),
    ]

    return exercises
