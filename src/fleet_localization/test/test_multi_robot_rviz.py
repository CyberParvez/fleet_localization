"""Two simultaneous operator views are process-private and robot-scoped."""
from pathlib import Path

from fleet_localization.operator_config import render_rviz


def test_two_rviz_renders_are_separate_and_do_not_cross_reference():
    template = Path(__file__).parents[1] / 'rviz' / 'robot_localization.rviz'
    first = render_rviz(template, 'robot1', '/robot1', 'robot1')
    second = render_rviz(template, 'robot2', '/robot2', 'robot2')
    try:
        assert first != second and first.parent == second.parent
        one, two = first.read_text(), second.read_text()
        assert 'Fixed Frame: robot1/map' in one and '/robot2' not in one
        assert 'Topic: /robot1/initialpose' in one
        assert 'Fixed Frame: robot2/map' in two and '/robot1' not in two
        assert 'Topic: /robot2/initialpose' in two
        assert template.read_text().count('@ROBOT@') > 0
        first.unlink()
        assert second.exists() and 'robot2/map' in second.read_text()
    finally:
        first.unlink(missing_ok=True); second.unlink(missing_ok=True)
