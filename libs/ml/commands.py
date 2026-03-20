from __future__ import annotations

COMMAND_VOCAB = ("lanefollow", "left", "right", "straight", "void")
COMMAND_TO_INDEX = {name: index for index, name in enumerate(COMMAND_VOCAB)}


def normalize_command(command: str | None) -> str:
    if command is None:
        return "void"
    normalized = str(command).strip().lower()
    if normalized in COMMAND_TO_INDEX:
        return normalized
    return "void"


def command_to_index(command: str | None) -> int:
    return COMMAND_TO_INDEX[normalize_command(command)]


def command_vocab_size() -> int:
    return len(COMMAND_VOCAB)
