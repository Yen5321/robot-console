from __future__ import annotations

from pathlib import Path
from typing import Any
from .parameters import validate_config


def load_config(path: str | Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required; run: pip install -r requirements.txt") from exc

    class UniqueLoader(yaml.SafeLoader):
        pass
    def unique_mapping(loader, node, deep=False):
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in result: raise ValueError(f"Duplicate YAML key: {key!r}, line {key_node.start_mark.line + 1}")
            result[key] = loader.construct_object(value_node, deep=deep)
        return result
    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(data, dict):
        raise ValueError(f"{config_path} must contain a YAML mapping")
    for section in ("arm", "camera", "network"):
        if not isinstance(data.get(section), dict):
            raise ValueError(f"missing mapping: {section}")
    validate_config(data)
    return data
