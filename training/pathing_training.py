from pathlib import Path
from dataclasses import dataclass, field
from math import pi

from rlbot.utils.game_state_util import GameState, BoostState, BallState, CarState, Physics, Vector3, Rotator
from rlbot.matchconfig.match_config import MatchConfig, PlayerConfig, Team
from rlbottraining.common_exercises.common_base_exercises import StrikerExercise
from rlbottraining.rng import SeededRandomNumberGenerator
from rlbottraining.match_configs import make_empty_match_config
from rlbottraining.grading.grader import Grader
from rlbottraining.training_exercise import TrainingExercise, Playlist

import training_util


def make_match_config_with_my_bot() -> MatchConfig:
    # Makes a config which only has our bot in it for now.
    # For more defails: https://youtu.be/uGFmOZCpel8?t=375
    match_config = make_empty_match_config()
    match_config.player_configs = [
        PlayerConfig.bot_config(
            Path(__file__).absolute().parent.parent / 'src' / 'bot.cfg',
            Team.BLUE
        ),
    ]
    return match_config


def add_my_bot_to_playlist(exercises: Playlist) -> Playlist:
    """
    Updates the match config for each excercise to include
    the bot from this project
    """
    for exercise in exercises:
        exercise.match_config = make_match_config_with_my_bot()
    return exercises


@dataclass
class OpenNet(StrikerExercise):
    def make_game_state(self, rng: SeededRandomNumberGenerator) -> GameState:

        return GameState(
            ball=BallState(physics=Physics(
                location=Vector3(0, 2000, 100),
                velocity=Vector3(0, 0, 1000),
                angular_velocity=Vector3(0, 0, 0))),
            cars={
                0: CarState(
                    physics=Physics(
                        location=Vector3(0, 0, 100),
                        rotation=Rotator(0, pi/2, 0),
                        velocity=Vector3(0, 0, 0),
                        angular_velocity=Vector3(0, 0, 0)),
                    jumped=False,
                    double_jumped=False,
                    boost_amount=100)
            },
            boosts={i: BoostState(0) for i in range(34)},
        )

@dataclass
class ToughAngle(StrikerExercise):
    def make_game_state(self, rng: SeededRandomNumberGenerator) -> GameState:
        return GameState(
            ball=BallState(physics=Physics(
                location=Vector3(3500, 4000, 100),
                velocity=Vector3(0, 0, 1000),
                angular_velocity=Vector3(0, 0, 0))),
            cars={
                0: CarState(
                    physics=Physics(
                        location=Vector3(1000, 2800, 100),
                        rotation=Rotator(0, -pi, 0),
                        velocity=Vector3(0, 0, 0),
                        angular_velocity=Vector3(0, 0, 0)),
                    jumped=False,
                    double_jumped=False,
                    boost_amount=100)
            },
            boosts={i: BoostState(0) for i in range(34)},
        )

@dataclass
class WallShot(StrikerExercise):
    def make_game_state(self, rng: SeededRandomNumberGenerator) -> GameState:
        return GameState(
            ball=BallState(physics=Physics(
                location=Vector3(2000, 1000, 100),
                velocity=Vector3(2000, rng.uniform(-500, 500), 0),
                angular_velocity=Vector3(0, 0, 0))),
            cars={
                0: CarState(
                    physics=Physics(
                        location=Vector3(1000, -1000, 100),
                        rotation=Rotator(0, 0, 0),
                        velocity=Vector3(0, 0, 0),
                        angular_velocity=Vector3(0, 0, 0)),
                    jumped=False,
                    double_jumped=False,
                    boost_amount=100)
            },
            boosts={i: BoostState(0) for i in range(34)},
        )

@dataclass
class SideShot(StrikerExercise):
    def make_game_state(self, rng: SeededRandomNumberGenerator) -> GameState:
        return GameState(
            ball=BallState(physics=Physics(
                location=Vector3(0, 2500, 100),
                velocity=Vector3(0, 0, rng.uniform(0, 1500)),
                angular_velocity=Vector3(0, 0, 0))),
            cars={
                0: CarState(
                    physics=Physics(
                        location=Vector3(rng.uniform(3000, 4000), rng.uniform(2000, 3000), 100),
                        rotation=Rotator(0, rng.uniform(0, pi), 0),
                        velocity=Vector3(0, 0, 0),
                        angular_velocity=Vector3(0, 0, 0)),
                    jumped=False,
                    double_jumped=False,
                    boost_amount=100)
            },
            boosts={i: BoostState(0) for i in range(34)},
        )

def make_default_playlist() -> Playlist:
    exercises = [
        Pathing('start perfectly center')
    ]
    return add_my_bot_to_playlist(exercises)
