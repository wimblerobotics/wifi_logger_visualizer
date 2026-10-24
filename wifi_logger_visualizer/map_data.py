import hashlib
import io
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import yaml


def _pgm_dimensions(data: bytes) -> Tuple[int, int]:
    if not data.startswith((b"P2", b"P5")):
        raise ValueError("Only P2 and P5 PGM map images are supported")
    tokens = []
    for line in data.splitlines():
        content = line.split(b"#", 1)[0]
        tokens.extend(content.split())
        if len(tokens) >= 4:
            break
    if len(tokens) < 4:
        raise ValueError("PGM header is incomplete")
    return int(tokens[1]), int(tokens[2])


def decode_pgm(data: bytes) -> np.ndarray:
    stream = io.BytesIO(data)

    def token() -> bytes:
        while True:
            byte = stream.read(1)
            while byte and byte.isspace():
                byte = stream.read(1)
            if byte == b"#":
                stream.readline()
                continue
            if not byte:
                raise ValueError("Unexpected end of PGM header")
            value = byte
            byte = stream.read(1)
            while byte and not byte.isspace():
                value += byte
                byte = stream.read(1)
            return value

    magic = token()
    width = int(token())
    height = int(token())
    maximum = int(token())
    if maximum <= 0 or maximum > 65535:
        raise ValueError("Unsupported PGM maximum value")
    if magic == b"P5":
        dtype = np.dtype(">u2") if maximum > 255 else np.dtype("u1")
        pixels = np.frombuffer(stream.read(), dtype=dtype, count=width * height)
    elif magic == b"P2":
        pixels = np.fromiter(
            (int(value) for value in stream.read().split()),
            dtype=np.uint16,
            count=width * height,
        )
    else:
        raise ValueError("Only P2 and P5 PGM map images are supported")
    if pixels.size != width * height:
        raise ValueError("PGM pixel data is incomplete")
    return pixels.reshape((height, width)).astype(np.float64) / maximum


def free_cell_centers(
    map_data: Dict[str, Any], cell_size: float
) -> list[Tuple[float, float]]:
    image = decode_pgm(map_data["image_data"])
    resolution = float(map_data["resolution"])
    origin_x = float(map_data["origin_x"])
    origin_y = float(map_data["origin_y"])
    min_cell_x = int(origin_x // cell_size)
    min_cell_y = int(origin_y // cell_size)
    max_cell_x = int((origin_x + map_data["width"] * resolution) // cell_size)
    max_cell_y = int((origin_y + map_data["height"] * resolution) // cell_size)
    result = []
    for cell_y in range(min_cell_y, max_cell_y + 1):
        for cell_x in range(min_cell_x, max_cell_x + 1):
            x, y = cell_center(cell_x, cell_y, cell_size)
            pixel_x = int((x - origin_x) / resolution)
            pixel_y = int(map_data["height"]) - 1 - int(
                (y - origin_y) / resolution
            )
            if not (
                0 <= pixel_x < int(map_data["width"])
                and 0 <= pixel_y < int(map_data["height"])
            ):
                continue
            occupancy = (
                image[pixel_y, pixel_x]
                if int(map_data["negate"])
                else 1.0 - image[pixel_y, pixel_x]
            )
            if occupancy < float(map_data["free_thresh"]):
                result.append((x, y))
    return result


def load_map(map_yaml_path: str) -> Dict[str, Any]:
    yaml_path = Path(map_yaml_path).expanduser().resolve()
    yaml_bytes = yaml_path.read_bytes()
    config = yaml.safe_load(yaml_bytes)
    if not isinstance(config, dict):
        raise ValueError(f"Map YAML is not an object: {yaml_path}")

    image_path = Path(str(config["image"])).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    image_data = image_path.read_bytes()
    width, height = _pgm_dimensions(image_data)
    origin = config.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("Map origin must contain x, y, and yaw")

    digest = hashlib.sha256()
    digest.update(yaml_bytes)
    digest.update(b"\0")
    digest.update(image_data)
    return {
        "content_hash": digest.hexdigest(),
        "source_yaml_path": str(yaml_path),
        "yaml_text": yaml_bytes.decode("utf-8"),
        "image_name": image_path.name,
        "image_data": image_data,
        "resolution": float(config["resolution"]),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "origin_yaw": float(origin[2]),
        "width": width,
        "height": height,
        "negate": int(config.get("negate", 0)),
        "occupied_thresh": float(config.get("occupied_thresh", 0.65)),
        "free_thresh": float(config.get("free_thresh", 0.25)),
    }


def cell_for_position(x: float, y: float, cell_size: float) -> Tuple[int, int]:
    if cell_size <= 0:
        raise ValueError("cell_size must be positive")
    return int(x // cell_size), int(y // cell_size)


def cell_center(cell_x: int, cell_y: int, cell_size: float) -> Tuple[float, float]:
    return ((cell_x + 0.5) * cell_size, (cell_y + 0.5) * cell_size)
