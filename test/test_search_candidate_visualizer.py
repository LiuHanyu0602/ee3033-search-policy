import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts'/'visualize_search_candidates.py'


def load_script():
    spec = importlib.util.spec_from_file_location('candidate_visualizer', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_log_parser_and_connectivity(tmp_path):
    module = load_script()
    log = tmp_path/'search.log'
    log.write_text(
        'Generated 8 candidate poses\n'
        'Candidate 3 failed: Nav2 returned an empty path\n'
        'Selected 2 by mlp; path=1.25m\n', encoding='utf-8')
    generated, failures, selected = module.parse_search_log(log)
    assert generated == 8
    assert failures == {3: ['Nav2 returned an empty path']}
    assert selected == [(2, 'mlp', 1.25)]
    mask = np.array([[1, 1, 0], [0, 1, 0], [1, 0, 1]], dtype=bool)
    reachable = module.reachable_component(mask, (0, 0))
    assert reachable.tolist() == [
        [True, True, False],
        [False, True, False],
        [False, False, False],
    ]


def test_visualizer_is_offline_and_non_actuating():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'generate_viewpoints_from_grid' in source
    assert 'Nav2Client' not in source
    assert 'rclpy' not in source
    assert '/cmd_vel' not in source
