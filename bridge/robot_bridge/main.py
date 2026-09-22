from __future__ import annotations

import argparse
import logging
import json

from .config import load_config
from .factory import create_arm, create_camera, create_laser
from .service import BridgeService
from .parameters import effective_config, safety_schema


def main() -> None:
    parser = argparse.ArgumentParser(description="Piper6/D435 robot bridge")
    parser.add_argument("--config", default="config.yaml", help="YAML configuration path")
    parser.add_argument("--check-config", action="store_true", help="Validate and print effective config without connecting hardware")
    parser.add_argument("--parameter-schema", action="store_true", help="Print safety parameter ranges and Chinese comments without connecting hardware")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    if args.parameter_schema:
        print(json.dumps(safety_schema(), ensure_ascii=False, indent=2))
        return
    config = load_config(args.config)
    if args.check_config:
        print(json.dumps({"bridge_build": "8.1-piper-l-cpv", "effective_config": effective_config(config)}, ensure_ascii=False, indent=2))
        return
    if config["arm"].get("driver") != "ros_servo":
        raise ValueError("v8 entrypoint requires ros_servo; legacy CAN writers must run only from the separate v7 release")
    config["network"].setdefault("audit_path", "logs/jog-v8.jsonl")
    arm = camera = laser = service = None
    try:
        arm = create_arm(config["arm"])
        camera = create_camera(config["camera"])
        laser = create_laser(config.get("laser"))
        service = BridgeService(
            arm, camera, laser, config["network"], config.get("telemetry"), configuration=config
        )
        service.start()
        service.wait()
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("stopping")
    finally:
        if service is not None:
            service.close()
        else:
            for device in (laser, camera, arm):
                if device is not None:
                    device.close()


