"""Render the one-robot RViz template into a process-temporary view."""
from pathlib import Path
import tempfile


def render_rviz(template: str | Path, robot: str, namespace: str, prefix: str) -> Path:
    text = Path(template).read_text(encoding='utf-8')
    for key, value in {'@ROBOT@': robot, '@NS@': namespace, '@PREFIX@': prefix}.items():
        text = text.replace(key, value)
    handle = tempfile.NamedTemporaryFile(mode='w', prefix=f'{robot}_localization_', suffix='.rviz', delete=False)
    with handle:
        handle.write(text)
    return Path(handle.name)
