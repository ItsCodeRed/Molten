from pathlib import Path

from rlbot.agents.base_loadout_generator import BaseLoadoutGenerator
from rlbot.matchconfig.loadout_config import LoadoutConfig


class SampleLoadoutGenerator(BaseLoadoutGenerator):
    def generate_loadout(self, player_index: int, team: int) -> LoadoutConfig:

        # generates proper loadout :)
        if player_index == 0 or player_index == 3 or player_index == 6:
            loadout = self.load_cfg_file(Path('appearance/octane_appearance.cfg'), team)
        elif player_index == 1 or player_index == 4 or player_index == 7:
            loadout = self.load_cfg_file(Path('appearance/dominus_appearance.cfg'), team)
        else:
            loadout = self.load_cfg_file(Path('appearance/fennec_appearance.cfg'), team)

        return loadout